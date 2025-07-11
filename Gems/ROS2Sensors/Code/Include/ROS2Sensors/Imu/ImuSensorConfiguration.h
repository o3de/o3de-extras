/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! A structure capturing configuration of a IMU sensor.
    struct ImuSensorConfiguration
    {
        AZ_TYPE_INFO(ImuSensorConfiguration, ImuSensorConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        //! Length of filter that removes numerical noise
        static constexpr int m_minFilterSize = 1;
        static constexpr int m_maxFilterSize = 200;
        int m_filterSize = 10;

        //! Include gravity acceleration
        bool m_includeGravity = true;

        //! Measure also absolute rotation
        bool m_absoluteRotation = true;

        AZ::Vector3 m_orientationVariance = AZ::Vector3::CreateZero();
        AZ::Vector3 m_angularVelocityVariance = AZ::Vector3::CreateZero();
        AZ::Vector3 m_linearAccelerationVariance = AZ::Vector3::CreateZero();
    };
} // namespace ROS2Sensors
