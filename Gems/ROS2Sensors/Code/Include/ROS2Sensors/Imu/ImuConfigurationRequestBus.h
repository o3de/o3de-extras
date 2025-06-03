/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>
#include <ROS2Sensors/Imu/ImuSensorConfiguration.h>

namespace ROS2Sensors
{
    //! Interface that allows to get and set Imu sensor's configuration.
    class ImuConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Returns the current configuration of the component.
        virtual const ImuSensorConfiguration GetConfiguration() = 0;

        //! Sets the configuration of the component.
        //! Each component should handle the configuration change without fully reinitializing the ROS2 publisher.
        //! This will allow to change the configuration of the component at runtime.
        //! Note: this method does not verify the configuration, so it is up to the caller to ensure that the configuration is valid.
        //! @param configuration The new configuration to set.
        virtual void SetConfiguration(const ImuSensorConfiguration& configuration) = 0;

        //! Gets the current filter size used for IMU data processing.
        virtual int GetFilterSize() = 0;

        //! Sets the filter size for IMU data processing.
        virtual void SetFilterSize(int filterSize) = 0;

        //! Checks if gravity is included in the IMU data.
        virtual bool GetIncludeGravity() = 0;

        //! Sets whether gravity should be included in the IMU data.
        virtual void SetIncludeGravity(bool includeGravity) = 0;

        //! Checks if absolute rotation is enabled for the IMU.
        virtual bool GetAbsoluteRotation() = 0;

        //! Sets whether absolute rotation should be enabled for the IMU.
        virtual void SetAbsoluteRotation(bool absoluteRotation) = 0;

        //! Gets the orientation variance for the IMU.
        virtual AZ::Vector3 GetOrientationVariance() = 0;

        //! Sets the orientation variance for the IMU.
        virtual void SetOrientationVariance(const AZ::Vector3& orientationVariance) = 0;

        //! Gets the angular velocity variance for the IMU.
        virtual AZ::Vector3 GetAngularVelocityVariance() = 0;

        //! Sets the angular velocity variance for the IMU.
        virtual void SetAngularVelocityVariance(const AZ::Vector3& angularVelocityVariance) = 0;

        //! Gets the linear acceleration variance for the IMU.
        virtual AZ::Vector3 GetLinearAccelerationVariance() = 0;

        //! Sets the linear acceleration variance for the IMU.
        virtual void SetLinearAccelerationVariance(const AZ::Vector3& linearAccelerationVariance) = 0;
    };

    using ImuConfigurationRequestBus = AZ::EBus<ImuConfigurationRequest>;
} // namespace ROS2Sensors
