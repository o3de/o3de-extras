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
#include <ROS2Sensors/Odometry/ROS2OdometryCovariance.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{

    //! A structure capturing configuration of a wheel odometry sensor.
    struct WheelOdometrySensorConfiguration
    {
        AZ_RTTI(WheelOdometrySensorConfiguration, ROS2WheelOdometryConfigurationTypeId);

        WheelOdometrySensorConfiguration() = default;
        virtual ~WheelOdometrySensorConfiguration() = default;

        static void Reflect(AZ::ReflectContext* context);

        ROS2OdometryCovariance m_poseCovariance;
        ROS2OdometryCovariance m_twistCovariance;
    };
} // namespace ROS2Sensors
