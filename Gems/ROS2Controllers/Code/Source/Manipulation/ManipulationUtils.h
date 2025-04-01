/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <ROS2/Manipulation/JointInfo.h>

namespace ROS2::Utils
{
    struct JointStateData
    {
        float position{ 0.f };
        float velocity{ 0.f };
        float effort{ 0.f };
    };

    //! Get the current joint state
    //! @param jointInfo Info of the joint we want to get data of.
    //! @return Data with the current joint state.
    JointStateData GetJointState(const JointInfo& jointInfo);
} // namespace ROS2::Utils
