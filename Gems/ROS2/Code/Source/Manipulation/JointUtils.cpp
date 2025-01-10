/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointUtils.h"

#include <ROS2/ROS2GemUtilities.h>
#include <Source/EditorArticulationLinkComponent.h>
#include <Source/EditorBallJointComponent.h>
#include <Source/EditorFixedJointComponent.h>
#include <Source/EditorHingeJointComponent.h>
#include <Source/EditorPrismaticJointComponent.h>

namespace ROS2::JointUtils
{

    bool HasNonFixedJoints(const AZ::Entity* entity)
    {
        const bool hasPrismaticJoints = Utils::HasComponentOfType(entity, PhysX::EditorPrismaticJointComponent::TYPEINFO_Uuid());
        const bool hasBallJoints = Utils::HasComponentOfType(entity, PhysX::EditorBallJointComponent::TYPEINFO_Uuid());
        const bool hasHingeJoints = Utils::HasComponentOfType(entity, PhysX::EditorHingeJointComponent::TYPEINFO_Uuid());
        const bool hasArticulations = Utils::HasComponentOfType(entity, PhysX::EditorArticulationLinkComponent::TYPEINFO_Uuid());
        const bool hasJoints = hasPrismaticJoints || hasBallJoints || hasHingeJoints || hasArticulations;

        return hasJoints;
    }
} // namespace ROS2::Utils
