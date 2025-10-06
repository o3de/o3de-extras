/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/Component/Entity.h>
#include <AzCore/std/optional.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <PhysX/ArticulationTypes.h>

namespace ROS2Controllers::Utils
{
    //! Retrieve root of articulation for given entity. If the entity is not part of an articulation, the invalid entity id is returned.
    //! @param entityId The entity to get the root of.
    //! @return The root of articulation.
    AZ::EntityId GetRootOfArticulation(AZ::EntityId entityId);

    //! Get handles to all the articulation links in an articulation tree.
    //! @param sceneHandle A handle to the scene.
    //! @param entityId Any entity in the articulation tree.
    //! @return Handles to all the articulation links in the tree.
    AZStd::unordered_map<AZ::EntityId, AzPhysics::SimulatedBodyHandle> GetSimulatedBodyHandles(
        AzPhysics::SceneHandle sceneHandle, AZ::EntityId entityId);

    //! Try to get a free articulation axis of an articulation link.
    //! @param entityId The entity with the articulation link.
    //! @return a free articulation axis if found and nullopt if not.
    AZStd::optional<PhysX::ArticulationJointAxis> TryGetFreeArticulationAxis(const AZ::EntityId& entityId);
} // namespace ROS2Controllers::Utils
