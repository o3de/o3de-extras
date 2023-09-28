/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>
#include <PhysX/ArticulationTypes.h>

namespace ROS2
{
    using JointPosition = float;
    using JointVelocity = float;
    using JointEffort = float;
    struct JointInfo
    {
        AZ_TYPE_INFO(JointInfo, "{2E33E4D0-78DD-436D-B3AB-F752E744F421}");
        static void Reflect(AZ::ReflectContext* context);

        bool m_isArticulation = false;
        PhysX::ArticulationJointAxis m_axis = PhysX::ArticulationJointAxis::Twist;
        AZ::EntityComponentIdPair m_entityComponentIdPair;
        JointPosition m_restPosition = 0.0f; //!< Keeps this position if no commands are given (for example, opposing gravity).
    };
    using ManipulationJoints = AZStd::unordered_map<AZStd::string, JointInfo>;

    struct JointInitialPosition
    {
        AZ_TYPE_INFO(JointInitialPosition, "{e81914ba-343a-478c-a4df-174d1f7b8a3d}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_name;
        float m_position;
        AZ::u32 m_index;
    };

    struct JointInitiaPositionComparator
    {
        AZ_TYPE_INFO(JointInitiaPositionComparator, "{9f585880-26a0-4dfc-ac4e-abf6160863d4}")

        bool operator()(const JointInitialPosition& a, const JointInitialPosition& b)
        {
            if (a.m_index != b.m_index)
            {
                return a.m_index < b.m_index;
            }
            return a.m_name < b.m_name;
        }
    };
} // namespace ROS2
