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

namespace ROS2 {

    //! A structure capturing configuration of a wheel odometry sensor.
    struct ROS2WheelOdometryConfiguration
    {
        AZ_RTTI(ROS2WheelOdometryConfiguration, "{9dc58d89-e674-4d7f-9ea9-afe3ae7fd2eb}");

        ROS2WheelOdometryConfiguration() = default;
        virtual ~ROS2WheelOdometryConfiguration() = default;

        static void Reflect(AZ::ReflectContext* context);

        ROS2OdometryCovariance m_poseCovariance;
        ROS2OdometryCovariance m_twistCovariance;
    };
}
