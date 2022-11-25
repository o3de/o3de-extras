/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "CameraSensor.h"

#include <sensor_msgs/image_encodings.hpp>

#include <AzCore/Math/MatrixUtils.h>

#include <Atom/RPI.Public/Base.h>
#include <Atom/RPI.Public/Pass/Specific/RenderToTexturePass.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/RenderPipeline.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Scene/SceneSystemInterface.h>

#include <Atom/RPI.Public/FeatureProcessorFactory.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <PostProcess/PostProcessFeatureProcessor.h>

#include <Atom/RPI.Public/Pass/PassFactory.h>

namespace ROS2
{
    namespace Internal
    {

        // maping from ATOM to ROS/OpenCV
        AZStd::unordered_map<AZ::RHI::Format, const char*> FormatMappings{
            { AZ::RHI::Format::R8G8B8A8_UNORM, sensor_msgs::image_encodings::RGBA8 },
            { AZ::RHI::Format::R16G16B16A16_UNORM, sensor_msgs::image_encodings::RGBA16 },
            { AZ::RHI::Format::R32G32B32A32_FLOAT, sensor_msgs::image_encodings::TYPE_32FC4 }, // Unsuported by RVIZ2
            { AZ::RHI::Format::R8_UNORM, sensor_msgs::image_encodings::MONO8 },
            { AZ::RHI::Format::R16_UNORM, sensor_msgs::image_encodings::MONO16 },
            { AZ::RHI::Format::R32_FLOAT, sensor_msgs::image_encodings::TYPE_32FC1 },

        };

    } // namespace Internal
    CameraSensorDescription::CameraSensorDescription(const AZStd::string& cameraName, float verticalFov, int width, int height)
        : m_verticalFieldOfViewDeg(verticalFov)
        , m_verticalFieldOfViewRad(AZ::DegToRad(m_verticalFieldOfViewDeg))
        , m_width(width)
        , m_height(height)
        , m_cameraName(cameraName)
        , m_aspectRatio(static_cast<float>(width) / static_cast<float>(height))
        , m_viewToClipMatrix(MakeViewToClipMatrix())
        , m_cameraIntrinsics(MakeCameraIntrinsics())
    {
        validateParameters();
    }

    AZ::Matrix4x4 CameraSensorDescription::MakeViewToClipMatrix() const
    {
        const float nearDist = 0.1f, farDist = 100.0f;
        AZ::Matrix4x4 localViewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(
            localViewToClipMatrix, AZ::DegToRad(m_verticalFieldOfViewDeg), m_aspectRatio, nearDist, farDist, true);
        return localViewToClipMatrix;
    }

    void CameraSensorDescription::validateParameters() const
    {
        AZ_Assert(
            m_verticalFieldOfViewDeg > 0.0f && m_verticalFieldOfViewDeg < 180.0f,
            "Vertical fov should be in range 0.0 < FoV < 180.0 degrees");
        AZ_Assert(!m_cameraName.empty(), "Camera name cannot be empty");
    }

    AZStd::array<double, 9> CameraSensorDescription::MakeCameraIntrinsics() const
    {
        //  Intrinsic camera matrix of the camera image is being created here
        //  It is based on other parameters available in the structure - they must be initialized before this function is called
        //  Matrix is row-major and has the following form:
        //  [fx  0 cx]
        //  [ 0 fy cy]
        //  [ 0  0  1]
        // Projects 3D points in the camera coordinate frame to 2D pixel
        // coordinates using the focal lengths (fx, fy) and principal point
        // (cx, cy).
        const auto w = static_cast<double>(m_width);
        const auto h = static_cast<double>(m_height);
        const double horizontalFoV = 2.0 * AZStd::atan(AZStd::tan(m_verticalFieldOfViewRad / 2.0) * m_aspectRatio);
        const double focalLengthX = w / (2.0 * AZStd::tan(horizontalFoV / 2.0));
        const double focalLengthY = h / (2.0 * AZStd::tan(m_verticalFieldOfViewRad / 2.0));
        return { focalLengthX, 0.0, w / 2.0, 0.0, focalLengthY, h / 2.0, 0.0, 0.0, 1.0 };
    }

    CameraSensor::CameraSensor(const CameraSensorDescription& cameraSensorDescription)
        : m_cameraSensorDescription(cameraSensorDescription)
    {
    }

    void CameraSensor::setupPasses()
    {
        AZ_TracePrintf("CameraSensor", "Initializing pipeline for %s", m_cameraSensorDescription.m_cameraName.c_str());

        AZ::Name viewName = AZ::Name("MainCamera");
        m_view = AZ::RPI::View::CreateView(viewName, AZ::RPI::View::UsageCamera);
        m_view->SetViewToClipMatrix(m_cameraSensorDescription.m_viewToClipMatrix);
        m_scene = AZ::RPI::RPISystemInterface::Get()->GetSceneByName(AZ::Name("Main"));

        const AZStd::string pipelineName = m_cameraSensorDescription.m_cameraName + "Pipeline" + getPipelineTypeName();

        AZ::RPI::RenderPipelineDescriptor pipelineDesc;
        pipelineDesc.m_mainViewTagName = "MainCamera";
        pipelineDesc.m_name = pipelineName;

        pipelineDesc.m_rootPassTemplate = getPipelineTemplateName();

        pipelineDesc.m_renderSettings.m_multisampleState = AZ::RPI::RPISystemInterface::Get()->GetApplicationMultisampleState();
        m_pipeline = AZ::RPI::RenderPipeline::CreateRenderPipeline(pipelineDesc);
        m_pipeline->RemoveFromRenderTick();

        if (auto renderToTexturePass = azrtti_cast<AZ::RPI::RenderToTexturePass*>(m_pipeline->GetRootPass().get()))
        {
            renderToTexturePass->ResizeOutput(m_cameraSensorDescription.m_width, m_cameraSensorDescription.m_height);
        }

        m_scene->AddRenderPipeline(m_pipeline);

        m_passHierarchy.push_back(pipelineName);
        m_passHierarchy.push_back("CopyToSwapChain");

        m_pipeline->SetDefaultView(m_view);
        AZ::RPI::ViewPtr targetView = m_scene->GetDefaultRenderPipeline()->GetDefaultView();
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
        AZ::Transform inverse = (cameraPose * kAtomToRos).GetInverse();
        m_view->SetWorldToViewMatrix(AZ::Matrix4x4::CreateFromQuaternionAndTranslation(inverse.GetRotation(), inverse.GetTranslation()));

        AZ::Render::FrameCaptureId captureId = AZ::Render::InvalidFrameCaptureId;

        m_pipeline->AddToRenderTickOnce();
        AZ::Render::FrameCaptureRequestBus::BroadcastResult(
            captureId,
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            m_passHierarchy,
            AZStd::string("Output"),
            callback,
            AZ::RPI::PassAttachmentReadbackOption::Output);
    }

    const CameraSensorDescription& CameraSensor::GetCameraSensorDescription() const
    {
        return m_cameraSensorDescription;
    }

    void CameraSensor::publishMassage(
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher,
        const AZ::Transform& cameraPose,
        const std_msgs::msg::Header& header)
    {
        RequestFrame(
            cameraPose,
            [header, publisher](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;
                const auto format = descriptor.m_format;
                AZ_Assert(Internal::FormatMappings.contains(format), "Unknown format in result %u", static_cast<uint32_t>(format));
                sensor_msgs::msg::Image message;
                message.encoding = Internal::FormatMappings.at(format);
                message.width = descriptor.m_size.m_width;
                message.height = descriptor.m_size.m_height;
                message.step = message.width * sensor_msgs::image_encodings::bitDepth(message.encoding) / 8 *
                    sensor_msgs::image_encodings::numChannels(message.encoding);
                message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                message.header = header;
                publisher->publish(message);
            });
    }

    CameraDepthSensor::CameraDepthSensor(const CameraSensorDescription& cameraSensorDescription)
        : CameraSensor(cameraSensorDescription)
    {
        setupPasses();
    }

    AZStd::string CameraDepthSensor::getPipelineTemplateName()
    {
        return "PipelineRenderToTextureROSDepth";
    };

    AZStd::string CameraDepthSensor::getPipelineTypeName()
    {
        return "Depth";
    };

    CameraColorSensor::CameraColorSensor(const CameraSensorDescription& cameraSensorDescription)
        : CameraSensor(cameraSensorDescription)
    {
        setupPasses();
    }

    AZStd::string CameraColorSensor::getPipelineTemplateName()
    {
        return "PipelineRenderToTextureROSColor";
    };

    AZStd::string CameraColorSensor::getPipelineTypeName()
    {
        return "Color";
    };

} // namespace ROS2
