/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "UrdfParser.h"
#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>

#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>

//! Common utils for Prefab Maker classes
namespace ROS2::PrefabMakerUtils
{
    //! Add required components to the entity.
    //! Calling this will ensure all the required (default) components are added.
    //! @param entityId entity to modify.
    void AddRequiredComponentsToEntity(AZ::EntityId entityId);

    //! For a given model, return path to its Assets.
    //! @param modelPath path to the model.
    //! @return path to assets directory for this model.
    AZ::IO::Path GetAzModelAssetPathFromModelPath(const AZ::IO::Path& modelPath);

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

    //! Get an Asset for a specified mesh given its path and mapping.
    //! @param sdfAssetsMapping mapping of SDF assets.
    //! @param sdfMeshPath a path to the mesh for which the Asset is requested.
    //! @return Asset for the mesh, if found in the mapping.
    AZStd::optional<Utils::AvailableAsset> GetAssetFromPath(
        const Utils::UrdfAssetMap& sdfAssetsMapping, const AZStd::string& sdfMeshPath);

    //! Get Asset from path. Version for std::string.
    //! @see GetAssetFromPath.
    AZStd::optional<Utils::AvailableAsset> GetAssetFromPath(const Utils::UrdfAssetMap& sdfAssetsMapping, const std::string& sdfMeshPath);
} // namespace ROS2::PrefabMakerUtils
