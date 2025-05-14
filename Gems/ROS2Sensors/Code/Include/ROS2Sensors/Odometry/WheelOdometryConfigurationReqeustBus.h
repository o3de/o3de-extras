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
#include <ROS2Sensors/Odometry/ROS2OdometryCovariance.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Interface that allows to get and set WheelOdometry sensor's configuration.
    class WheelOdometryConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Get the pose covariance.
        virtual ROS2OdometryCovariance GetPoseCovariance() const = 0;

        //! Set the pose covariance.
        virtual void SetPoseCovariance(const ROS2OdometryCovariance& covariance) = 0;

        //! Get the twist covariance.
        virtual ROS2OdometryCovariance GetTwistCovariance() const = 0;

        //! Set the twist covariance.
        virtual void SetTwistCovariance(const ROS2OdometryCovariance& covariance) = 0;
    };

    using WheelOdometryConfigurationRequestBus = AZ::EBus<WheelOdometryConfigurationRequest>;
} // namespace ROS2Sensors
