/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SensorsEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <GNSS/ROS2GNSSSensorComponent.h>
#include <Imu/ROS2ImuSensorComponent.h>
#include <Lidar/ROS2Lidar2DSensorComponent.h>
#include <Lidar/ROS2LidarSensorComponent.h>

namespace ROS2Sensors
{
    AZ_COMPONENT_IMPL(
        ROS2SensorsEditorSystemComponent, "ROS2SensorsEditorSystemComponent", ROS2SensorsEditorSystemComponentTypeId, BaseSystemComponent);

    void ROS2SensorsEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SensorsEditorSystemComponent, ROS2SensorsSystemComponent>()->Version(0);
        }
    }

    ROS2SensorsEditorSystemComponent::ROS2SensorsEditorSystemComponent()
    {
        if (ROS2SensorsEditorInterface::Get() == nullptr)
        {
            ROS2SensorsEditorInterface::Register(this);
        }
    }

    ROS2SensorsEditorSystemComponent::~ROS2SensorsEditorSystemComponent()
    {
        if (ROS2SensorsEditorInterface::Get() == this)
        {
            ROS2SensorsEditorInterface::Unregister(this);
        }
    }

    void ROS2SensorsEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2SensorsEditorService"));
    }

    void ROS2SensorsEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2SensorsEditorService"));
    }

    void ROS2SensorsEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void ROS2SensorsEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ROS2SensorsEditorSystemComponent::Activate()
    {
        ROS2SensorsSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        ROS2SensorsEditorRequestBus::Handler::BusConnect();
    }

    void ROS2SensorsEditorSystemComponent::Deactivate()
    {
        ROS2SensorsEditorRequestBus::Handler::BusDisconnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ROS2SensorsSystemComponent::Deactivate();
    }

    AZ::Component* ROS2SensorsEditorSystemComponent::CreateROS2GnssSensorComponent(
        AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration)
    {
        return CreateComponent<ROS2GNSSSensorComponent>(entity, sensorConfiguration);
    };

    AZ::Component* ROS2SensorsEditorSystemComponent::CreateROS2CameraSensorComponent(
        AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration)
    {
        return CreateComponent<ROS2CameraSensorEditorComponent>(entity, sensorConfiguration, cameraConfiguration);
    };

    AZ::Component* ROS2SensorsEditorSystemComponent::CreateROS2ImuSensorComponent(
        AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration, const ImuSensorConfiguration& imuConfiguration)
    {
        return CreateComponent<ROS2ImuSensorComponent>(entity, sensorConfiguration, imuConfiguration);
    };

    AZ::Component* ROS2SensorsEditorSystemComponent::CreateROS2LidarSensorComponent(
        AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration)
    {
        return CreateComponent<ROS2LidarSensorComponent>(entity, sensorConfiguration, lidarConfiguration);
    };

    AZ::Component* ROS2SensorsEditorSystemComponent::CreateROS2Lidar2DSensorComponent(
        AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidar2DConfiguration)
    {
        return CreateComponent<ROS2Lidar2DSensorComponent>(entity, sensorConfiguration, lidar2DConfiguration);
    };

    AZ::Component* ROS2SensorsEditorSystemComponent::CreateROS2OdometrySensorComponent(
        AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration)
    {
        return nullptr;
    };
} // namespace ROS2Sensors
