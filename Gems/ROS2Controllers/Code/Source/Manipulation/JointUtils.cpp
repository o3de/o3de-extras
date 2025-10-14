/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointUtils.h"

#include <Source/EditorArticulationLinkComponent.h>
#include <Source/EditorBallJointComponent.h>
#include <Source/EditorHingeJointComponent.h>
#include <Source/EditorPrismaticJointComponent.h>

namespace ROS2Controllers::JointUtils
{

    bool HasNonFixedJoints(const AZ::Entity* entity)
    {
        const auto* prismaticJoint = entity->FindComponent<PhysX::EditorPrismaticJointComponent>();
        const auto* ballJoint = entity->FindComponent<PhysX::EditorBallJointComponent>();
        const auto* hingeJoint = entity->FindComponent<PhysX::EditorHingeJointComponent>();
        const auto* articulation = entity->FindComponent<PhysX::EditorArticulationLinkComponent>();
        const bool hasJoints = prismaticJoint || ballJoint || hingeJoint || articulation;

        return hasJoints;
    }
} // namespace ROS2Controllers::JointUtils
