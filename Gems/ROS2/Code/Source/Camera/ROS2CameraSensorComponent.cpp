/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CameraSensorComponent.h"
#include "CameraUtilities.h"
#include "ROS2/Frame/ROS2FrameBus.h"
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <sensor_msgs/distortion_models.hpp>

namespace ROS2
{
    ROS2CameraSensorComponent::ROS2CameraSensorComponent(
        const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration)
        : m_cameraConfiguration(cameraConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()->Version(4)->Field(
                "CameraSensorConfig", &ROS2CameraSensorComponent::m_cameraConfiguration);
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        if (m_cameraConfiguration.m_colorCamera && m_cameraConfiguration.m_depthCamera)
        {
            AddImageSource<CameraRGBDSensor>();
        }
        else if (m_cameraConfiguration.m_colorCamera)
        {
            AddImageSource<CameraColorSensor>();
        }
        else if (m_cameraConfiguration.m_depthCamera)
        {
            AddImageSource<CameraDepthSensor>();
        }

        bool hasFrameComponent = false;
        ROS2FrameComponentBus::EventResult(hasFrameComponent, GetEntityId(), &ROS2FrameComponentBus::Events::IsFrame);
        AZ_Assert(hasFrameComponent, "Entity has no ROS2FrameComponent");
        ROS2FrameComponentBus::EventResult(m_frameName, GetEntityId(), &ROS2FrameComponentBus::Events::GetFrameID);
        ROS2::CameraCalibrationRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensor.reset();
        ROS2SensorComponent::Deactivate();
        ROS2::CameraCalibrationRequestBus::Handler::BusDisconnect(GetEntityId());
    }

    AZ::Matrix3x3 ROS2CameraSensorComponent::GetCameraMatrix() const
    {
        return CameraUtils::MakeCameraIntrinsics(
            m_cameraConfiguration.m_width, m_cameraConfiguration.m_height, m_cameraConfiguration.m_verticalFieldOfViewDeg);
    };

    int ROS2CameraSensorComponent::GetWidth() const
    {
        return m_cameraConfiguration.m_width;
    };

    int ROS2CameraSensorComponent::GetHeight() const
    {
        return m_cameraConfiguration.m_height;
    };

    float ROS2CameraSensorComponent::GetVerticalFOV() const
    {
        return m_cameraConfiguration.m_verticalFieldOfViewDeg;
    };

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
        const AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();
        const auto timestamp = ROS2Interface::Get()->GetROSTimestamp();
        if (m_cameraSensor)
        {
            std_msgs::msg::Header messageHeader;
            messageHeader.stamp = timestamp;
            messageHeader.frame_id = m_frameName.c_str();
            m_cameraSensor->RequestMessagePublication(transform, messageHeader);
        }
    }

    AZStd::string ROS2CameraSensorComponent::GetCameraNameFromFrame(const AZ::Entity* entity) const
    {
        bool hasFrameComponent = false;
        ROS2FrameComponentBus::EventResult(hasFrameComponent, entity->GetId(), &ROS2FrameComponentBus::Events::IsFrame);
        AZ_Assert(hasFrameComponent, "Entity %s has no ROS2CameraSensorComponent", entity->GetName().c_str());
        if (hasFrameComponent)
        {
            AZStd::string cameraName;
            ROS2FrameComponentBus::EventResult(cameraName, entity->GetId(), &ROS2FrameComponentBus::Events::GetFrameID);
            AZStd::replace(cameraName.begin(), cameraName.end(), '/', '_');
            return cameraName;
        }
        return AZStd::string{};
    }
} // namespace ROS2
