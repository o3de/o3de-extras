/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Controllers/ROS2ControllersTypeIds.h>
#include <ROS2Controllers/Sensors/ROS2OdometryCovariance.h>

namespace ROS2Controllers
{

    //! A structure capturing configuration of a wheel odometry sensor.
    struct WheelOdometrySensorConfiguration
    {
        AZ_RTTI(WheelOdometrySensorConfiguration, WheelOdometrySensorConfigurationTypeId);

        WheelOdometrySensorConfiguration() = default;
        virtual ~WheelOdometrySensorConfiguration() = default;

        static void Reflect(AZ::ReflectContext* context);

        ROS2OdometryCovariance m_poseCovariance;
        ROS2OdometryCovariance m_twistCovariance;
    };
} // namespace ROS2Controllers
