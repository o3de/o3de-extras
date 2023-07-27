/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! A structure capturing configuration of a IMU sensor.
    struct ImuSensorConfiguration
    {
        AZ_TYPE_INFO(ImuSensorConfiguration, "{6788e84f-b985-4413-8e2b-46fbfb667c95}");
        static void Reflect(AZ::ReflectContext* context);

        //! Length of filter that removes numerical noise
        int m_filterSize = 10;
        int m_minFilterSize = 1;
        int m_maxFilterSize = 200;

        //! Include gravity acceleration
        bool m_includeGravity = true;

        //! Measure also absolute rotation
        bool m_absoluteRotation = true;

        AZ::Vector3 m_orientationVariance = AZ::Vector3::CreateZero();
        AZ::Vector3 m_angularVelocityVariance = AZ::Vector3::CreateZero();
        AZ::Vector3 m_linearAccelerationVariance = AZ::Vector3::CreateZero();
    };
} // namespace ROS2
