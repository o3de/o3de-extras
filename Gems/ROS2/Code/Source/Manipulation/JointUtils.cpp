/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointUtils.h"

#include <ROS2/ROS2GemUtilities.h>

namespace ROS2::Utils
{

    bool CheckIfEntityHasNonFixedJoints(const AZ::Entity* entity)
    {
        const bool hasPrismaticJoints = Utils::CheckIfEntityHasComponentOfType(
            entity, AZ::Uuid("{607B246E-C2DB-4D43-ABFA-A5A3994867C5}")); // Physx::EditorPrismaticJointComponent
        const bool hasBallJoints = Utils::CheckIfEntityHasComponentOfType(
            entity, AZ::Uuid("{3D770685-9271-444D-B8AE-783B652C0986}")); // Physx::EditorBallJointComponent
        const bool hasHingeJoints = Utils::CheckIfEntityHasComponentOfType(
            entity, AZ::Uuid("{AF60FD48-4ADC-4C8C-8D3A-A4F7AE63C74D}")); // Physx::EditorHingeJointComponent
        const bool hasArticulations = Utils::CheckIfEntityHasComponentOfType(
            entity, AZ::Uuid("{7D23169B-3214-4A32-ABFC-FCCE6E31F2CF}")); // Physx::EditorArticulationLinkComponent
        const bool hasJoints = hasPrismaticJoints || hasBallJoints || hasHingeJoints || hasArticulations;

        return hasJoints;
    }
} // namespace ROS2::Utils
