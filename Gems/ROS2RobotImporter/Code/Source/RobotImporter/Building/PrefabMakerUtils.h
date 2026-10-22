/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/Assets/AssetTypes.h>

#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include <RobotImporter/ReferencedAssetsRequestBus.h>
#include <gz/math/Pose3.hh>

//! Common utils for Prefab Maker classes
namespace ROS2RobotImporter::PrefabMakerUtils
{
    //! Set the transform for an entity.
    //! @param origin pose for the entity to set.
    //! @param entityId entity which will be modified.
    void SetEntityTransformLocal(const gz::math::Pose3d& origin, AZ::EntityId entityId);

    //! Create a prefab entity in a hierarchy. The new entity will not yet be active.
    //! @param parentEntityId id of parent entity for this new entity.
    //! Passing an invalid id would get the entity in the current context (for example, an entity which is currently open in the Editor).
    //! @param name name for the new entity.
    //! @return a result which is either a created prefab entity or an error.
    AzToolsFramework::Prefab::PrefabEntityResult CreateEntity(AZ::EntityId parentEntityId, const AZStd::string& name);

    //! Set the parent entity for an entity. The entity being attached to the parent entity is expected to be inactive.
    //! NOTE: This uses the world transform of the entity when updating the transform
    //! The world location of the entity will not change
    //! @param entityId the id for entity that needs a parent.
    //! @param parentEntityId the id for the parent entity.
    void SetEntityParent(AZ::EntityId entityId, AZ::EntityId parentEntityId);

    //! Set the parent entity for an entity. The entity being attached to the parent is expected to be inactive.
    //! NOTE: This uses the local transform of the entity when updating the transform
    //! and therefore allows the entity to relocate based on the parent world transform
    //! @param entityId the id for entity that needs a parent.
    //! @param parentEntityId the id for the parent entity.
    void SetEntityParentRelative(AZ::EntityId entityId, AZ::EntityId parentEntityId);

    //! Create an entity name from arguments.
    //! @param rootName root of entity's name.
    //! @param type type of entity, depending on corresponding SDF tag. For example, "visual".
    //! @param index index of entity, useful when multiple visuals or colliders are present for a single link.
    //! @return entity name, for example "robotBumper_visual_1".
    AZStd::string MakeEntityName(const AZStd::string& rootName, const AZStd::string& type, size_t index = 0);
} // namespace ROS2RobotImporter::PrefabMakerUtils
