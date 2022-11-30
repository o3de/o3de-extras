/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "RobotImporter/Utils/SourceAssetsStorage.h"
#include "UrdfParser.h"
#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>

#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>

//! Common utils for Prefab Maker classes
namespace ROS2::PrefabMakerUtils
{
    void AddRequiredComponentsToEntity(AZ::EntityId entityId);
    AZ::IO::Path GetAzModelAssetPathFromModelPath(const AZ::IO::Path& modelPath);
    void SetEntityTransformLocal(const urdf::Pose& origin, AZ::EntityId entityId);
    AzToolsFramework::Prefab::PrefabEntityResult CreateEntity(AZ::EntityId parentEntityId, const AZStd::string& name);
    AzToolsFramework::Prefab::PrefabOperationResult RemoveEntityWithDescendants(AZ::EntityId parentEntityId);
    AzToolsFramework::EntityIdList GetColliderChildren(AZ::EntityId parentEntityId);
    bool HasCollider(AZ::EntityId entityId);
    AZStd::string MakeEntityName(const AZStd::string& rootName, const AZStd::string& type, size_t index = 0);

    AZStd::optional<Utils::AvailableAsset> GetAssetFromPath(
        const Utils::UrdfAssetMap& urdfAssetsMapping, const AZStd::string& urdfMeshPath);

    AZStd::optional<Utils::AvailableAsset> GetAssetFromPath(const Utils::UrdfAssetMap& urdfAssetsMapping, const std::string& urdfMeshPath);

} // namespace ROS2::PrefabMakerUtils
