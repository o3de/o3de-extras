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
#include <ROS2Controllers/Sensors/ROS2OdometryCovariance.h>
#include <ROS2Controllers/Sensors/WheelOdometrySensorConfiguration.h>

namespace ROS2Controllers
{
    //! Interface that allows to get and set WheelOdometry sensor's configuration.
    class WheelOdometryConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Returns the current configuration of the component.
        virtual const WheelOdometrySensorConfiguration GetConfiguration() = 0;

        //! Sets the configuration of the component.
        //! Each component should handle the configuration change without fully reinitializing the ROS2 publisher.
        //! This will allow to change the configuration of the component at runtime.
        //! @param configuration The new configuration to set.
        virtual void SetConfiguration(const WheelOdometrySensorConfiguration& configuration) = 0;

        //! Get the pose covariance.
        virtual ROS2OdometryCovariance GetPoseCovariance() = 0;

        //! Set the pose covariance.
        virtual void SetPoseCovariance(const ROS2OdometryCovariance& covariance) = 0;

        //! Get the twist covariance.
        virtual ROS2OdometryCovariance GetTwistCovariance() = 0;

        //! Set the twist covariance.
        virtual void SetTwistCovariance(const ROS2OdometryCovariance& covariance) = 0;
    };

    using WheelOdometryConfigurationRequestBus = AZ::EBus<WheelOdometryConfigurationRequest>;
} // namespace ROS2Controllers
