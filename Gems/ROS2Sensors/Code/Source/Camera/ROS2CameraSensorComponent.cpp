/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CameraSensorComponent.h"
#include "CameraUtilities.h"
#include <ROS2/Frame/ROS2FrameComponent.h>

namespace ROS2Sensors
{
    ROS2CameraSensorComponent::ROS2CameraSensorComponent(
        const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration)
        : m_cameraConfiguration(cameraConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        CameraSensorConfiguration::Reflect(context);
        ROS2SensorComponentBase<ROS2::TickBasedSource>::Reflect(context);

        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, SensorBaseType>()->Version(5)->Field(
                "CameraSensorConfig", &ROS2CameraSensorComponent::m_cameraConfiguration);
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();

        SetCameraSensorConfiguration();

        const auto* component = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(component, "Entity has no ROS2FrameComponent");
        m_frameName = component->GetFrameID();
        CameraConfigurationRequestBus::Handler::BusConnect(GetEntityId());

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                FrequencyTick();
            });
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        StopSensor();
        m_cameraSensor.reset();
        CameraConfigurationRequestBus::Handler::BusDisconnect(GetEntityId());
        ROS2SensorComponentBase::Deactivate();
    }

    void ROS2CameraSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ROS2CameraSensorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2CameraSensor"));
    }

    void ROS2CameraSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2CameraSensor"));
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        if (!m_cameraSensor)
        {
            return;
        }

        const AZ::Transform& transform = GetEntity()->GetTransform()->GetWorldTM();
        const auto timestamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();

        std_msgs::msg::Header messageHeader;
        messageHeader.stamp = timestamp;
        messageHeader.frame_id = m_frameName.c_str();
        m_cameraSensor->RequestMessagePublication(transform, messageHeader);
    }

    AZStd::string ROS2CameraSensorComponent::GetCameraNameFromFrame(const AZ::Entity* entity) const
    {
        const auto* component = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        AZ_Assert(component, "Entity %s has no ROS2CameraSensorComponent", entity->GetName().c_str());
        if (component)
        {
            AZStd::string cameraName = component->GetFrameID();
            AZStd::replace(cameraName.begin(), cameraName.end(), '/', '_');
            return cameraName;
        }
        return AZStd::string{};
    }

    void ROS2CameraSensorComponent::SetCameraSensorConfiguration()
    {
        if (m_cameraConfiguration.m_colorCamera && m_cameraConfiguration.m_depthCamera)
        {
            SetImageSource<CameraRGBDSensor>();
        }
        else if (m_cameraConfiguration.m_colorCamera)
        {
            SetImageSource<CameraColorSensor>();
        }
        else if (m_cameraConfiguration.m_depthCamera)
        {
            SetImageSource<CameraDepthSensor>();
        }
    }

    AZ::Matrix3x3 ROS2CameraSensorComponent::GetCameraMatrix() const
    {
        return CameraUtils::MakeCameraIntrinsics(
            m_cameraConfiguration.m_width, m_cameraConfiguration.m_height, m_cameraConfiguration.m_verticalFieldOfViewDeg);
    }

    float ROS2CameraSensorComponent::GetVerticalFOV() const
    {
        return m_cameraConfiguration.m_verticalFieldOfViewDeg;
    }

    void ROS2CameraSensorComponent::SetVerticalFOV(float value)
    {
        m_cameraConfiguration.m_verticalFieldOfViewDeg = value;
    }

    int ROS2CameraSensorComponent::GetWidth() const
    {
        return m_cameraConfiguration.m_width;
    }

    void ROS2CameraSensorComponent::SetWidth(int value)
    {
        m_cameraConfiguration.m_width = value;
    }

    int ROS2CameraSensorComponent::GetHeight() const
    {
        return m_cameraConfiguration.m_height;
    }

    void ROS2CameraSensorComponent::SetHeight(int value)
    {
        m_cameraConfiguration.m_height = value;
    }

    bool ROS2CameraSensorComponent::IsColorCamera() const
    {
        return m_cameraConfiguration.m_colorCamera;
    }

    void ROS2CameraSensorComponent::SetColorCamera(bool value)
    {
        m_cameraConfiguration.m_colorCamera = value;
    }

    bool ROS2CameraSensorComponent::IsDepthCamera() const
    {
        return m_cameraConfiguration.m_depthCamera;
    }

    void ROS2CameraSensorComponent::SetDepthCamera(bool value)
    {
        m_cameraConfiguration.m_depthCamera = value;
    }

    float ROS2CameraSensorComponent::GetNearClipDistance() const
    {
        return m_cameraConfiguration.m_nearClipDistance;
    }

    void ROS2CameraSensorComponent::SetNearClipDistance(float value)
    {
        m_cameraConfiguration.m_nearClipDistance = value;
    }

    float ROS2CameraSensorComponent::GetFarClipDistance() const
    {
        return m_cameraConfiguration.m_farClipDistance;
    }

    void ROS2CameraSensorComponent::SetFarClipDistance(float value)
    {
        m_cameraConfiguration.m_farClipDistance = value;
    }
} // namespace ROS2Sensors
