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

namespace ROS2
{
    namespace Internal
    {
        /// @FormatMappings - contains the mapping from RHI to ROS image encodings. List of supported
        /// ROS image encodings lives in `sensor_msgs/image_encodings.hpp`
        /// We are not including `image_encodings.hpp` since it uses exceptions.
        AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, "rgba8" },     { AZ::RHI::Format::R16G16B16A16_UNORM, "rgba16" },
            { AZ::RHI::Format::R32G32B32A32_FLOAT, "32FC4" }, // Unsupported by RVIZ2
            { AZ::RHI::Format::R8_UNORM, "mono8" },           { AZ::RHI::Format::R16_UNORM, "mono16" },
            { AZ::RHI::Format::R32_FLOAT, "32FC1" },
        };

        /// @BitDepth - contains the mapping from RHI to size used in `step` size computation.
        /// It is some equivalent to `bitDepth()` function from `sensor_msgs/image_encodings.hpp`
        AZStd::unordered_map<AZ::RHI::Format, int> BitDepth{
            { AZ::RHI::Format::R8G8B8A8_UNORM, 4 * sizeof(uint8_t) },
            { AZ::RHI::Format::R16G16B16A16_UNORM, 4 * sizeof(uint16_t) },
            { AZ::RHI::Format::R32G32B32A32_FLOAT, 4 * sizeof(float) }, // Unsupported by RVIZ2
            { AZ::RHI::Format::R8_UNORM, sizeof(uint8_t) },
            { AZ::RHI::Format::R16_UNORM, sizeof(uint16_t) },
            { AZ::RHI::Format::R32_FLOAT, sizeof(float) },
        };
    } // namespace Internal
<<<<<<< HEAD

    CameraSensorDescription::CameraSensorDescription(const AZStd::string& cameraName, const CameraConfiguration& configuration)
        : m_cameraConfiguration(configuration)
=======
    CameraSensorDescription::CameraSensorDescription(const AZStd::string& cameraName, float verticalFov, int width, int height, AZ::EntityId entityId)
        : m_verticalFieldOfViewDeg(verticalFov)
        , m_width(width)
        , m_height(height)
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
        , m_cameraName(cameraName)
        , m_aspectRatio(static_cast<float>(configuration.m_width) / static_cast<float>(configuration.m_height))
        , m_viewToClipMatrix(MakeViewToClipMatrix())
        , m_cameraIntrinsics(MakeCameraIntrinsics())
        , m_entityId(entityId)
    {
        ValidateParameters();
    }

    AZ::Matrix4x4 CameraSensorDescription::MakeViewToClipMatrix() const
    {
        const float nearDist = 0.1f, farDist = 100.0f;
        AZ::Matrix4x4 localViewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(
            localViewToClipMatrix, AZ::DegToRad(m_cameraConfiguration.m_verticalFieldOfViewDeg), m_aspectRatio, nearDist, farDist, true);
        return localViewToClipMatrix;
    }

    void CameraSensorDescription::ValidateParameters() const
    {
        AZ_Assert(
            m_cameraConfiguration.m_verticalFieldOfViewDeg > 0.0f && m_cameraConfiguration.m_verticalFieldOfViewDeg < 180.0f,
            "Vertical fov should be in range 0.0 < FoV < 180.0 degrees");
        AZ_Assert(!m_cameraName.empty(), "Camera name cannot be empty");
    }

    AZStd::array<double, 9> CameraSensorDescription::MakeCameraIntrinsics() const
    {
        /* Intrinsic camera matrix of the camera image is being created here
           It is based on other parameters available in the structure - they must be initialized before this function is called
            Matrix is row-major and has the following form:
            [fx  0 cx]
            [ 0 fy cy]
            [ 0  0  1]
           Projects 3D points in the camera coordinate frame to 2D pixel
           coordinates using the focal lengths (fx, fy) and principal point
           (cx, cy).
       */
        const auto w = static_cast<double>(m_cameraConfiguration.m_width);
        const auto h = static_cast<double>(m_cameraConfiguration.m_height);
        const double verticalFieldOfView = AZ::DegToRad(m_cameraConfiguration.m_verticalFieldOfViewDeg);
        const double horizontalFoV = 2.0 * AZStd::atan(AZStd::tan(verticalFieldOfView / 2.0) * m_aspectRatio);
        const double focalLengthX = w / (2.0 * AZStd::tan(horizontalFoV / 2.0));
        const double focalLengthY = h / (2.0 * AZStd::tan(verticalFieldOfView / 2.0));
        return { focalLengthX, 0.0, w / 2.0, 0.0, focalLengthY, h / 2.0, 0.0, 0.0, 1.0 };
    }

    CameraSensor::CameraSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId)
        : m_cameraSensorDescription(cameraSensorDescription)
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

<<<<<<< HEAD
        m_pipelineName = AZStd::string::format(
            "%sPipeline%s%s", m_cameraSensorDescription.m_cameraName.c_str(), GetPipelineTypeName().c_str(), m_entityId.ToString().c_str());
=======
        m_pipelineName = AZStd::string::format("%sPipeline%s%s",m_cameraSensorDescription.m_cameraName.c_str(), GetPipelineTypeName().c_str(),
                                               m_cameraSensorDescription.m_entityId.ToString().c_str() );
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
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
        const AZ::Transform inverse = (cameraPose * AtomToRos).GetInverse();
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

    void CameraSensor::RequestMessagePublication(
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher,
        const AZ::Transform& cameraPose,
        const std_msgs::msg::Header& header)
    {
        RequestFrame(
            cameraPose,
            [header, publisher, entityId = m_entityId](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
<<<<<<< HEAD
                if (result.m_state != AZ::RPI::AttachmentReadback::ReadbackState::Success)
                {
                    return;
                }

                const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                const auto format = descriptor.m_format;
                AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
                sensor_msgs::msg::Image message;
                message.encoding = Internal::FormatMappings.at(format);
                message.width = descriptor.m_size.m_width;
                message.height = descriptor.m_size.m_height;
                message.step = message.width * Internal::BitDepth.at(format);
                message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                message.header = header;
                bool registeredPostProcessingSupportsEncoding = false;
                CameraPostProcessingRequestBus::EventResult(
                    registeredPostProcessingSupportsEncoding,
                    entityId,
                    &CameraPostProcessingRequests::SupportsFormat,
                    AZStd::string(Internal::FormatMappings.at(format)));
                if (registeredPostProcessingSupportsEncoding)
                {
                    CameraPostProcessingRequestBus::Event(entityId, &CameraPostProcessingRequests::ApplyPostProcessing, message);
                }
                publisher->publish(message);
=======
                if (result.m_state == AZ::RPI::AttachmentReadback::ReadbackState::Success)
                {
                    const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                    const auto format = descriptor.m_format;
                    AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
                    sensor_msgs::msg::Image message;
                    message.encoding = Internal::FormatMappings.at(format);
                    message.width = descriptor.m_size.m_width;
                    message.height = descriptor.m_size.m_height;
                    message.step = message.width * Internal::BitDepth.at(format);
                    message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                    message.header = header;
                    publisher->publish(message);
                }
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
            });
    }

    void CameraSensor::RequestMessagePublication(
        AZStd::span<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> publishers,
        const AZ::Transform& cameraPose,
        const std_msgs::msg::Header& header)
    {
        if (!publishers.empty())
        {
            RequestMessagePublication(publishers.front(), cameraPose, header);
        }
    }

<<<<<<< HEAD
    CameraDepthSensor::CameraDepthSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId)
        : CameraSensor(cameraSensorDescription, entityId)
=======
    CameraDepthSensor::CameraDepthSensor(const CameraSensorDescription& cameraSensorDescription)
        : CameraSensor(cameraSensorDescription)
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
    {
        SetupPasses();
    }

    AZStd::string CameraDepthSensor::GetPipelineTemplateName() const
    {
        return "PipelineRenderToTextureROSDepth";
    };

    AZStd::string CameraDepthSensor::GetPipelineTypeName() const
    {
        return "Depth";
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

    AZStd::string CameraColorSensor::GetPipelineTypeName() const
    {
        return "Color";
    };

    CameraRGBDSensor::CameraRGBDSensor(const CameraSensorDescription& cameraSensorDescription)
        : CameraColorSensor(cameraSensorDescription)
    {
    }

    void CameraRGBDSensor::ReadBackDepth(
        AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback)
    {
            AZ::Render::FrameCaptureOutcome captureOutcome;
            AZStd::vector<AZStd::string> passHierarchy{m_pipelineName,"DepthPrePass"};
            AZ::Render::FrameCaptureRequestBus::BroadcastResult(
                captureOutcome,
                &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
                callback,
                passHierarchy,
                AZStd::string("DepthLinear"),
                AZ::RPI::PassAttachmentReadbackOption::Output);
    }

    void CameraRGBDSensor::RequestMessagePublication(
        AZStd::span<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> publishers,
        const AZ::Transform& cameraPose,
        const std_msgs::msg::Header& header)
    {
        AZ_Assert(publishers.size()==2, "RequestMessagePublication for CameraRGBDSensor should be called with exactly two publishers");
        const auto publisherDepth = publishers.back();
        ReadBackDepth(
            [header, publisherDepth](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                if (result.m_state == AZ::RPI::AttachmentReadback::ReadbackState::Success)
                {
                    const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                    const auto format = descriptor.m_format;
                    AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
                    sensor_msgs::msg::Image message;
                    message.encoding = Internal::FormatMappings.at(format);
                    message.width = descriptor.m_size.m_width;
                    message.height = descriptor.m_size.m_height;
                    message.step = message.width * Internal::BitDepth.at(format);
                    message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                    message.header = header;
                    publisherDepth->publish(message);
                }
            });
        CameraSensor::RequestMessagePublication(publishers,cameraPose,header);
    }

} // namespace ROS2
