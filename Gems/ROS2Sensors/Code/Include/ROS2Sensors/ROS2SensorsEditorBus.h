/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2Sensors/Camera/CameraSensorConfiguration.h>
#include <ROS2Sensors/Imu/ImuSensorConfiguration.h>
#include <ROS2Sensors/Lidar/LidarSensorConfiguration.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <AzCore/Component/Component.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2Sensors
{
    //! Interface for creating ROS2 sensor components.
    //! This interface provides methods to create various types of ROS2 sensor components, such as GNSS, Camera, IMU, Lidar, and Odometry
    //! sensors. It is intended to be used by external tools that create entities, e.g., when importing robots from SDF, URDF, or OpenUSD.
    class ROS2SensorsEditorRequests
    {
    public:
        AZ_RTTI(ROS2SensorsEditorRequests, ROS2SensorsEditorRequestsTypeId);
        virtual ~ROS2SensorsEditorRequests() = default;

        //! Create a new ROS2 GNSS sensor component.
        //! @param entity The entity to which the sensor component will be added.
        //! @param sensorConfiguration The general configuration for the sensor.
        //! @param gnssConfiguration The specific configuration parameters for the GNSS sensor.
        //! @return A pointer to the newly created AZ::Component representing the GNSS sensor (or nullptr if failed).
        virtual AZ::Component* CreateROS2GnssSensorComponent(AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration) = 0;

        //! Create a new ROS2 Camera sensor component.
        //! @param entity The entity to which the sensor component will be added.
        //! @param sensorConfiguration The general configuration for the sensor.
        //! @param cameraConfiguration The specific configuration parameters for the Camera sensor.
        //! @return A pointer to the newly created AZ::Component representing the Camera sensor (or nullptr if failed).
        virtual AZ::Component* CreateROS2CameraSensorComponent(
            AZ::Entity& entity,
            const ROS2::SensorConfiguration& sensorConfiguration,
            const CameraSensorConfiguration& cameraConfiguration) = 0;

        //! Create a new ROS2 IMU sensor component.
        //! @param entity The entity to which the sensor component will be added.
        //! @param sensorConfiguration The general configuration for the sensor.
        //! @param imuConfiguration The specific configuration parameters for the IMU sensor.
        //! @return A pointer to the newly created AZ::Component representing the IMU sensor (or nullptr if failed).
        virtual AZ::Component* CreateROS2ImuSensorComponent(
            AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration, const ImuSensorConfiguration& imuConfiguration) = 0;

        //! Create a new ROS2 Lidar sensor component.
        //! @param entity The entity to which the sensor component will be added.
        //! @param sensorConfiguration The general configuration for the sensor.
        //! @param lidarConfiguration The specific configuration parameters for the Lidar sensor.
        //! @return A pointer to the newly created AZ::Component representing the Lidar sensor (or nullptr if failed).
        virtual AZ::Component* CreateROS2LidarSensorComponent(
            AZ::Entity& entity,
            const ROS2::SensorConfiguration& sensorConfiguration,
            const LidarSensorConfiguration& lidarConfiguration) = 0;

        //! Create a new ROS2 2D Lidar sensor component.
        //! @param entity The entity to which the sensor component will be added.
        //! @param sensorConfiguration The general configuration for the sensor.
        //! @param lidar2DConfiguration The specific configuration parameters for the 2D Lidar sensor.
        //! @return A pointer to the newly created AZ::Component representing the 2D Lidar sensor (or nullptr if failed).
        virtual AZ::Component* CreateROS2Lidar2DSensorComponent(
            AZ::Entity& entity,
            const ROS2::SensorConfiguration& sensorConfiguration,
            const LidarSensorConfiguration& lidar2DConfiguration) = 0;

        //! Create a new ROS2 Odometry sensor component.
        //! @param entity The entity to which the sensor component will be added.
        //! @param sensorConfiguration The general configuration for the sensor.
        //! @return A pointer to the newly created AZ::Component representing the Odometry sensor (or nullptr if failed).
        virtual AZ::Component* CreateROS2OdometrySensorComponent(
            AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration) = 0;
    };

    class ROS2SensorsEditorBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2SensorsEditorInterface = AZ::Interface<ROS2SensorsEditorRequests>;

} // namespace ROS2Sensors
