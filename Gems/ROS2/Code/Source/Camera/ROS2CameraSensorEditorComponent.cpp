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
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("camera_image_color", CameraConstants::ImageMessageType, CameraConstants::ColorImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("camera_image_depth", CameraConstants::ImageMessageType, CameraConstants::DepthImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("color_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::ColorInfoConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            MakeTopicConfigurationPair("depth_camera_info", CameraConstants::CameraInfoMessageType, CameraConstants::DepthInfoConfig));
    }

    ROS2CameraSensorEditorComponent::ROS2CameraSensorEditorComponent(
        const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration)
        : m_sensorConfiguration(sensorConfiguration)
        , m_cameraSensorConfiguration(cameraConfiguration)
    {
    }

    void ROS2CameraSensorEditorComponent::Reflect(AZ::ReflectContext* context)
    {
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
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
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
        if (!m_sensorConfiguration.m_visualize)
        {
            return;
        }
        const AZ::u32 stateBefore = debugDisplay.GetState();
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        const float distance = m_cameraSensorConfiguration.m_farClipDistance * 0.1f;
        const float tangent = static_cast<float>(tan(0.5f * AZ::DegToRad(m_cameraSensorConfiguration.m_verticalFieldOfViewDeg)));

        float height = distance * tangent;
        float width = height * m_cameraSensorConfiguration.m_width / m_cameraSensorConfiguration.m_height;

        AZ::Vector3 farPoints[4];
        farPoints[0] = AZ::Vector3(width, height, distance);
        farPoints[1] = AZ::Vector3(-width, height, distance);
        farPoints[2] = AZ::Vector3(-width, -height, distance);
        farPoints[3] = AZ::Vector3(width, -height, distance);

        AZ::Vector3 nearPoints[4];
        nearPoints[0] = farPoints[0].GetNormalizedSafe() * m_cameraSensorConfiguration.m_nearClipDistance;
        nearPoints[1] = farPoints[1].GetNormalizedSafe() * m_cameraSensorConfiguration.m_nearClipDistance;
        nearPoints[2] = farPoints[2].GetNormalizedSafe() * m_cameraSensorConfiguration.m_nearClipDistance;
        nearPoints[3] = farPoints[3].GetNormalizedSafe() * m_cameraSensorConfiguration.m_nearClipDistance;

        // dimension of drawing
        const float arrowRise = 0.11f;
        const float arrowSize = 0.05f;

        const AZ::Vector3 pt0(0, 0, 0);
        const auto ptz = AZ::Vector3::CreateAxisZ(0.2f);
        const auto pty = AZ::Vector3::CreateAxisY(0.2f);
        const auto ptx = AZ::Vector3::CreateAxisX(0.2f);

        debugDisplay.PushMatrix(transform);

        debugDisplay.SetColor(AZ::Color(0.f, 1.f, 1.f, 1.f));
        debugDisplay.DrawLine(nearPoints[0], farPoints[0]);
        debugDisplay.DrawLine(nearPoints[1], farPoints[1]);
        debugDisplay.DrawLine(nearPoints[2], farPoints[2]);
        debugDisplay.DrawLine(nearPoints[3], farPoints[3]);
        debugDisplay.DrawPolyLine(nearPoints, AZ_ARRAY_SIZE(nearPoints));
        debugDisplay.DrawPolyLine(farPoints, AZ_ARRAY_SIZE(farPoints));

        // up-arrow drawing
        const AZ::Vector3 pa1(-arrowSize * height, -arrowRise * width, 1.f);
        const AZ::Vector3 pa2(arrowSize * height, -arrowRise * width, 1.f);
        const AZ::Vector3 pa3(0, (-arrowRise - arrowSize) * width, 1.f);

        debugDisplay.SetColor(AZ::Color(0.f, 0.6f, 1.f, 1.f));
        debugDisplay.SetLineWidth(1);
        debugDisplay.DrawLine(pa1, pa2);
        debugDisplay.DrawLine(pa2, pa3);
        debugDisplay.DrawLine(pa3, pa1);

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

        debugDisplay.PopMatrix();

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

} // namespace ROS2
