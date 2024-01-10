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
#include <AzCore/std/string/string.h>
#include <ROS2/Manipulation/JointInfo.h>

namespace ROS2
{
    struct JointNamePositionPair
    {
        AZ_TYPE_INFO(JointNamePositionPair, "{f6bc855d-c82f-45f9-a8d7-3a3cdbd7e9fe}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_name;
        JointPosition m_position = 0.0f;
    };
} // namespace ROS2
