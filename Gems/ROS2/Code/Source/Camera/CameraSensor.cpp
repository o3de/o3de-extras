/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "CameraSensor.h"
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Math/MatrixUtils.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Scene/SceneSystemInterface.h>
#include <PostProcess/PostProcessFeatureProcessor.h>

#include <sensor_msgs/distortion_models.hpp>

namespace ROS2
{
    namespace Internal
    {
        /// @FormatMappings - contains the mapping from RHI to ROS image encodings. List of supported
        /// ROS image encodings lives in `sensor_msgs/image_encodings.hpp`
        /// We are not including `image_encodings.hpp` since it uses exceptions.
        const AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, "rgba8" },
            { AZ::RHI::Format::R32_FLOAT, "32FC1" },
        };

        /// @BitDepth - contains the mapping from RHI to size used in `step` size computation.
        /// It is some equivalent to `bitDepth()` function from `sensor_msgs/image_encodings.hpp`
        const AZStd::unordered_map<AZ::RHI::Format, int> BitDepth{
            { AZ::RHI::Format::R8G8B8A8_UNORM, 4 * sizeof(uint8_t) },
            { AZ::RHI::Format::R32_FLOAT, sizeof(float) },
        };

        //! Create a CameraImage message from the read-back result and a header.
        sensor_msgs::msg::Image CreateImageMessageFromReadBackResult(
            const AZ::EntityId& entityId, const AZ::RPI::AttachmentReadback::ReadbackResult& result, const std_msgs::msg::Header& header)
        {
            const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
            const auto format = descriptor.m_format;
            AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
            sensor_msgs::msg::Image imageMessage;
            imageMessage.encoding = Internal::FormatMappings.at(format);
            imageMessage.width = descriptor.m_size.m_width;
            imageMessage.height = descriptor.m_size.m_height;
            imageMessage.step = imageMessage.width * Internal::BitDepth.at(format);
            imageMessage.data =
                std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
            imageMessage.header = header;
            CameraPostProcessingRequestBus::Event(entityId, &CameraPostProcessingRequests::ApplyPostProcessing, imageMessage);
            return imageMessage;
        }

        //! Prepare a CameraInfo message from sensor description and a header.
        sensor_msgs::msg::CameraInfo CreateCameraInfoMessage(
            const CameraSensorDescription& cameraDescription, const std_msgs::msg::Header& header)
        {
            const auto& cameraIntrinsics = cameraDescription.m_cameraIntrinsics;
            sensor_msgs::msg::CameraInfo cameraInfo;
            cameraInfo.width = cameraDescription.m_cameraConfiguration.m_width;
            cameraInfo.height = cameraDescription.m_cameraConfiguration.m_height;
            cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            [[maybe_unused]] constexpr size_t expectedMatrixSize = 9;
            AZ_Assert(cameraInfo.k.size() == expectedMatrixSize, "camera matrix should have %d elements", expectedMatrixSize);
            cameraInfo.k = { cameraIntrinsics.GetElement(0, 0),
                             0,
                             cameraIntrinsics.GetElement(0, 2),
                             0,
                             cameraIntrinsics.GetElement(1, 1),
                             cameraIntrinsics.GetElement(1, 2),
                             0,
                             0,
                             1 };
            cameraInfo.p = { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2], 0, cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5], 0,
                             cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8], 0 };
            cameraInfo.header = header;
            return cameraInfo;
        }

        AZStd::string PipelineNameFromChannelType(CameraSensorDescription::CameraChannelType channel)
        {
            static const AZStd::unordered_map<CameraSensorDescription::CameraChannelType, AZStd::string> channelNameMap = {
                { CameraSensorDescription::CameraChannelType::RGB, "Color" }, { CameraSensorDescription::CameraChannelType::DEPTH, "Depth" }
            };
            AZ_Assert(channelNameMap.count(channel) == 1, "Channel type not found in the dictionary!");
            return channelNameMap.at(channel);
        }
    } // namespace Internal

    CameraSensor::CameraSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId)
        : m_cameraPublishers(cameraSensorDescription)
        , m_cameraSensorDescription(cameraSensorDescription)
        , m_entityId(entityId)
    {
    }

    void CameraSensor::SetupPasses()
    {
        AZ_TracePrintf("CameraSensor", "Initializing pipeline for %s\n", m_cameraSensorDescription.m_cameraName.c_str());

        const AZ::Name viewName = AZ::Name("MainCamera");
        m_view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
        m_view->SetViewToClipMatrix(m_cameraSensorDescription.m_viewToClipMatrix);
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        auto cameraPipelineTypeName = Internal::PipelineNameFromChannelType(GetChannelType());

        m_pipelineName = AZStd::string::format(
            "%sPipeline%s%s",
            m_cameraSensorDescription.m_cameraName.c_str(),
            cameraPipelineTypeName.c_str(),
            m_entityId.ToString().c_str());
        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_mainViewTagName = "MainCamera";
        pipelineDesc.m_name = m_pipelineName;

        pipelineDesc.m_rootPassTemplate = GetPipelineTemplateName();

        pipelineDesc.m_renderSettings.m_multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
        m_pipeline->RemoveFromRenderTick();

        if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(m_pipeline->GetRootPass().get()))
        {
            renderToTexturePass->ResizeOutput(
                m_cameraSensorDescription.m_cameraConfiguration.m_width, m_cameraSensorDescription.m_cameraConfiguration.m_height);
        }

        m_scene->AddRenderPipeline(m_pipeline);

        m_passHierarchy.push_back(m_pipelineName);
        m_passHierarchy.push_back("CopyToSwapChain");

        m_pipeline->SetDefaultView(m_view);
        const AZ::RPI::ViewPtr targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
        if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
        {
            fp->SetViewAlias(m_view, targetView);
        }
    }

    CameraSensor::~CameraSensor()
    {
        if (m_scene)
        {
            if (auto* fp = m_scene->GetFeatureProcessor<AZ::Render::PostProcessFeatureProcessor>())
            {
                fp->RemoveViewAlias(m_view);
            }
            m_scene->RemoveRenderPipeline(m_pipeline->GetId());
            m_scene = nullptr;
        }
        m_passHierarchy.clear();
        m_pipeline.reset();
        m_view.reset();
    }

    void CameraSensor::RequestFrame(
        const AZ::Transform& cameraPose, AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback)
    {
        const AZ::Transform cameraPoseNoScaling =
            AZ::Transform::CreateFromQuaternionAndTranslation(cameraPose.GetRotation(), cameraPose.GetTranslation());
        const AZ::Transform inverse = (cameraPoseNoScaling * AtomToRos).GetInverse();
        m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(), inverse.GetTranslation()));

        AZ::Render::FrameCaptureOutcome captureOutcome;

        m_pipeline->AddToRenderTickOnce();
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            captureOutcome,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            callback,
            m_passHierarchy,
            AZStd::string("Output"),
            AZ::RPI::PassAttachmentReadbackOption::Output);

        AZ_Error(
            "CameraSensor",
            captureOutcome.IsSuccess(),
            "Frame capture initialization failed. %s",
            captureOutcome.GetError().m_errorMessage.c_str());
    }

    const CameraSensorDescription& CameraSensor::GetCameraSensorDescription() const
    {
        return m_cameraSensorDescription;
    }

    void CameraSensor::RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header)
    {
        auto imagePublisher = m_cameraPublishers.GetImagePublisher(GetChannelType());
        auto infoPublisher = m_cameraPublishers.GetInfoPublisher(GetChannelType());
        if (!imagePublisher || !infoPublisher)
        {
            AZ_Error("CameraSensor::RequestMessagePublication", false, "Missing publisher for the Camera sensor");
            return;
        }

        auto infoMessage = Internal::CreateCameraInfoMessage(m_cameraSensorDescription, header);
        RequestFrame(
            cameraPose,
            [header, imagePublisher, infoPublisher, infoMessage, entityId = m_entityId](
                const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                if (result.m_state != AZ::RPI::AttachmentReadback::ReadbackState::Success)
                {
                    return;
                }

                auto imageMessage = Internal::CreateImageMessageFromReadBackResult(entityId, result, header);
                imagePublisher->publish(imageMessage);
                infoPublisher->publish(infoMessage);
            });
    }

    CameraDepthSensor::CameraDepthSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId)
        : CameraSensor(cameraSensorDescription, entityId)
    {
        SetupPasses();
    }

    AZStd::string CameraDepthSensor::GetPipelineTemplateName() const
    {
        return "PipelineRenderToTextureROSDepth";
    };

    CameraSensorDescription::CameraChannelType CameraDepthSensor::GetChannelType() const
    {
        return CameraSensorDescription::CameraChannelType::DEPTH;
    };

    CameraColorSensor::CameraColorSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId)
        : CameraSensor(cameraSensorDescription, entityId)
    {
        SetupPasses();
    }

    AZStd::string CameraColorSensor::GetPipelineTemplateName() const
    {
        return "PipelineRenderToTextureROSColor";
    };

    CameraSensorDescription::CameraChannelType CameraColorSensor::GetChannelType() const
    {
        return CameraSensorDescription::CameraChannelType::RGB;
    };

    CameraRGBDSensor::CameraRGBDSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId)
        : CameraColorSensor(cameraSensorDescription, entityId)
    {
    }

    void CameraRGBDSensor::ReadBackDepth(AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback)
    {
        AZ::Render::FrameCaptureOutcome captureOutcome;
        AZStd::vector<AZStd::string> passHierarchy{ m_pipelineName, "DepthPrePass" };
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            captureOutcome,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            callback,
            passHierarchy,
            AZStd::string("DepthLinear"),
            AZ::RPI::PassAttachmentReadbackOption::Output);
    }

    void CameraRGBDSensor::RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header)
    {
        auto imagePublisher = m_cameraPublishers.GetImagePublisher(CameraSensorDescription::CameraChannelType::DEPTH);
        auto infoPublisher = m_cameraPublishers.GetInfoPublisher(CameraSensorDescription::CameraChannelType::DEPTH);
        if (!imagePublisher || !infoPublisher)
        {
            AZ_Error("CameraRGBDSensor::RequestMessagePublication", false, "Missing publisher for the Camera sensor");
            return;
        }

        auto infoMessage = Internal::CreateCameraInfoMessage(m_cameraSensorDescription, header);
        // Process the Depth part.
        ReadBackDepth(
            [header, imagePublisher, infoPublisher, infoMessage, entityId = m_entityId](
                const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                if (result.m_state != AZ::RPI::AttachmentReadback::ReadbackState::Success)
                {
                    return;
                }
                auto imageMessage = Internal::CreateImageMessageFromReadBackResult(entityId, result, header);
                imagePublisher->publish(imageMessage);
                infoPublisher->publish(infoMessage);
            });

        // Process the Color part.
        CameraSensor::RequestMessagePublication(cameraPose, header);
    }
} // namespace ROS2
