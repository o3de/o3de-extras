/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CameraSensorEditorComponent.h"
#include "CameraConstants.h"
#include "CameraUtilities.h"
#include "ROS2CameraSensorComponent.h"
#include <AzCore/Component/TransformBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>

namespace ROS2
{
    ROS2CameraSensorEditorComponent::ROS2CameraSensorEditorComponent()
    {
        m_sensorConfiguration.m_frequency = 10;
        InitializeSensorTopics();
    }

    ROS2CameraSensorEditorComponent::ROS2CameraSensorEditorComponent(
        const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration)
        : m_sensorConfiguration(sensorConfiguration)
        , m_cameraSensorConfiguration(cameraConfiguration)
    {
        InitializeSensorTopics();
    }

    void ROS2CameraSensorEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        CameraSensorConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2CameraSensorEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(4)
                ->Field("CameraSensorConfig", &ROS2CameraSensorEditorComponent::m_cameraSensorConfiguration)
                ->Field("SensorConfig", &ROS2CameraSensorEditorComponent::m_sensorConfiguration);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2CameraSensorEditorComponent>("ROS2 Camera Sensor", "[Camera component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CameraSensorEditorComponent::m_sensorConfiguration,
                        "Sensor configuration",
                        "Sensor configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CameraSensorEditorComponent::m_cameraSensorConfiguration,
                        "Camera configuration",
                        "Camera configuration.");
            }
        }
    }

    void ROS2CameraSensorEditorComponent::Activate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(this->GetEntityId());
        AzToolsFramework::Components::EditorComponentBase::Activate();
        ROS2::CameraCalibrationRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2CameraSensorEditorComponent::Deactivate()
    {
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        ROS2::CameraCalibrationRequestBus::Handler::BusDisconnect(GetEntityId());
    }

    void ROS2CameraSensorEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ROS2CameraSensorEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2CameraSensor"));
    }

    void ROS2CameraSensorEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2CameraSensor"));
    }

    void ROS2CameraSensorEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<ROS2::ROS2CameraSensorComponent>(m_sensorConfiguration, m_cameraSensorConfiguration);
    }

    AZ::Matrix3x3 ROS2CameraSensorEditorComponent::GetCameraMatrix() const
    {
        return CameraUtils::MakeCameraIntrinsics(
            m_cameraSensorConfiguration.m_width,
            m_cameraSensorConfiguration.m_height,
            m_cameraSensorConfiguration.m_verticalFieldOfViewDeg);
    };

    int ROS2CameraSensorEditorComponent::GetWidth() const
    {
        return m_cameraSensorConfiguration.m_width;
    };

    int ROS2CameraSensorEditorComponent::GetHeight() const
    {
        return m_cameraSensorConfiguration.m_height;
    };

    float ROS2CameraSensorEditorComponent::GetVerticalFOV() const
    {
        return m_cameraSensorConfiguration.m_verticalFieldOfViewDeg;
    };

    void ROS2CameraSensorEditorComponent::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        if (!m_sensorConfiguration.m_visualise)
        {
            return;
        }
        const AZ::u32 stateBefore = debugDisplay.GetState();
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        // dimension of drawing
        const float arrowRise = 1.1f;
        const float arrowSize = 0.5f;
        const float frustumScale = 0.1f;

        transform.SetUniformScale(frustumScale);
        debugDisplay.DepthTestOff();
        const float vertical = 0.5f * AZStd::tan((AZ::DegToRad(m_cameraSensorConfiguration.m_verticalFieldOfViewDeg * 0.5f)));
        const float horizontal = m_cameraSensorConfiguration.m_height * vertical / m_cameraSensorConfiguration.m_width;

        // frustum drawing
        const AZ::Vector3 p1(-vertical, -horizontal, 1.f);
        const AZ::Vector3 p2(vertical, -horizontal, 1.f);
        const AZ::Vector3 p3(vertical, horizontal, 1.f);
        const AZ::Vector3 p4(-vertical, horizontal, 1.f);
        const AZ::Vector3 p0(0, 0, 0);
        const AZ::Vector3 py(0.1, 0, 0);

        const auto pt1 = transform.TransformPoint(p1);
        const auto pt2 = transform.TransformPoint(p2);
        const auto pt3 = transform.TransformPoint(p3);
        const auto pt4 = transform.TransformPoint(p4);
        const auto pt0 = transform.TransformPoint(p0);
        const auto ptz = transform.TransformPoint(AZ::Vector3::CreateAxisZ(0.2f));
        const auto pty = transform.TransformPoint(AZ::Vector3::CreateAxisY(0.2f));
        const auto ptx = transform.TransformPoint(AZ::Vector3::CreateAxisX(0.2f));

        debugDisplay.SetColor(AZ::Color(0.f, 1.f, 1.f, 1.f));
        debugDisplay.SetLineWidth(1);
        debugDisplay.DrawLine(pt1, pt2);
        debugDisplay.DrawLine(pt2, pt3);
        debugDisplay.DrawLine(pt3, pt4);
        debugDisplay.DrawLine(pt4, pt1);
        debugDisplay.DrawLine(pt1, pt0);
        debugDisplay.DrawLine(pt2, pt0);
        debugDisplay.DrawLine(pt3, pt0);
        debugDisplay.DrawLine(pt4, pt0);

        // up-arrow drawing
        const AZ::Vector3 pa1(-arrowSize * vertical, -arrowRise * horizontal, 1.f);
        const AZ::Vector3 pa2(arrowSize * vertical, -arrowRise * horizontal, 1.f);
        const AZ::Vector3 pa3(0, (-arrowRise - arrowSize) * horizontal, 1.f);
        const auto pat1 = transform.TransformPoint(pa1);
        const auto pat2 = transform.TransformPoint(pa2);
        const auto pat3 = transform.TransformPoint(pa3);

        debugDisplay.SetColor(AZ::Color(0.f, 0.6f, 1.f, 1.f));
        debugDisplay.SetLineWidth(1);
        debugDisplay.DrawLine(pat1, pat2);
        debugDisplay.DrawLine(pat2, pat3);
        debugDisplay.DrawLine(pat3, pat1);

        // coordinate system drawing
        debugDisplay.SetColor(AZ::Color(1.f, 0.f, 0.f, 1.f));
        debugDisplay.SetLineWidth(2);
        debugDisplay.DrawLine(ptx, pt0);

        debugDisplay.SetColor(AZ::Color(0.f, 1.f, 0.f, 1.f));
        debugDisplay.SetLineWidth(2);
        debugDisplay.DrawLine(pty, pt0);

        debugDisplay.SetColor(AZ::Color(0.f, 0.f, 1.f, 1.f));
        debugDisplay.SetLineWidth(2);
        debugDisplay.DrawLine(ptz, pt0);

        debugDisplay.SetState(stateBefore);
    }

    AZStd::pair<AZStd::string, TopicConfiguration> ROS2CameraSensorEditorComponent::MakeTopicConfigurationPair(
        const AZStd::string& topic, const AZStd::string& messageType, const AZStd::string& configName) const
    {
        TopicConfiguration config;
        config.m_topic = topic;
        config.m_type = messageType;
        return AZStd::make_pair(configName, config);
    }

    void ROS2CameraSensorEditorComponent::InitializeSensorTopics()
    {
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("camera_image_color", CameraConstants::ImageMessageType, CameraConstants::ColorImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("camera_image_depth", CameraConstants::ImageMessageType, CameraConstants::DepthImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("color_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::ColorInfoConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("depth_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::DepthInfoConfig));
    }

} // namespace ROS2
