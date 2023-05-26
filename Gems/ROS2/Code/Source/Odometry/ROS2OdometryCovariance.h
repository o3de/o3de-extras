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
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <array>

namespace ROS2
{
    struct ROS2OdometryCovariance
    {
        AZ_TYPE_INFO(ROS2OdometryCovariance, "{db015832-f621-11ed-b67e-0242ac120002}");

        static void Reflect(AZ::ReflectContext* context);

        std::array<double, 36> GetRosCovariance() const;

        AZ::Vector3 m_linearCovariance = AZ::Vector3::CreateZero();
        AZ::Vector3 m_angularCovariance = AZ::Vector3::CreateZero();
    };
} // namespace ROS2
