/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Controllers/Controllers/PidConfiguration.h>

namespace ROS2Controllers
{
    //! Configuration class for RigidBodyTwistControlComponent
    class RigidBodyTwistControlComponentConfig
    {
    public:
        enum class PhysicalApi
        {
            Velocity,
            Kinematic,
            Force
        };

        AZ_TYPE_INFO(RigidBodyTwistControlComponentConfig, "{8F2E9C4B-1D5A-4B7C-9E8F-3A6B5C7D8E9F}");

        static void Reflect(AZ::ReflectContext* context);

        // Visibility functions for edit context
        [[nodiscard]] AZ::u32 GetPidControllersVisibility() const;
        [[nodiscard]] AZStd::string GetNote() const;

        //! Pid controllers for linear velocities - default config for each axis
        AZStd::array<PidConfiguration, 3> m_linerControllers = { PidConfiguration(500, 50, 0, 0, 0, false, false),
                                                                 PidConfiguration(500, 50, 0, 0, 0, false, false),
                                                                 PidConfiguration(0, 0, 0, 0, 0, false, false) };

        //! Pid controllers for angular velocities - default config for each axis
        AZStd::array<PidConfiguration, 3> m_angularControllers = { PidConfiguration(0, 0, 0, 0, 0, false, false),
                                                                   PidConfiguration(0, 0, 0, 0, 0, false, false),
                                                                   PidConfiguration(500, 50, 0, 0, 0, false, false) };
        PhysicalApi m_physicalApi = PhysicalApi::Velocity; //!< API to use for applying velocities
    };
} // namespace ROS2Controllers
