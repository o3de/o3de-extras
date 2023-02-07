/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/Utils/TypeConversions.h"
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/string/conversions.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorShapeColliderComponent.h>

namespace ROS2::PrefabMakerUtils
{
    AZ::IO::Path GetAzModelAssetPathFromModelPath(const AZ::IO::Path& modelPath)
    {
        bool assetFound = false;
        AZ::Data::AssetInfo assetInfo;
        AZ::IO::Path watchDir;
        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
            assetFound,
            &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
            modelPath.c_str(),
            assetInfo,
            watchDir.Native());

        if (!assetFound)
        {
            AZ_Error("PrefabMakerUtils", false, "Could not find model asset for %s", modelPath.c_str());
            return {};
        }

        auto assetPath = AZ::IO::Path(assetInfo.m_relativePath).ReplaceExtension("azmodel");
        AZStd::to_lower(assetPath.Native().begin(), assetPath.Native().end());

        return assetPath;
    }

    void SetEntityTransformLocal(const urdf::Pose& origin, AZ::EntityId entityId)
    {
        urdf::Vector3 urdfPosition = origin.position;
        urdf::Rotation urdfRotation = origin.rotation;
        AZ::Quaternion azRotation = URDF::TypeConversions::ConvertQuaternion(urdfRotation);
        AZ::Vector3 azPosition = URDF::TypeConversions::ConvertVector3(urdfPosition);
        AZ::Transform tf(azPosition, azRotation, 1.0f);

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();

        if (!transformInterface)
        {
            AZ_Error("SetEntityTransform", false, "Missing Transform component!");
            return;
        }
        transformInterface->SetLocalTM(tf);
    }

    AzToolsFramework::Prefab::PrefabEntityResult CreateEntity(AZ::EntityId parentEntityId, const AZStd::string& name)
    {
        auto* prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
        auto createEntityResult = prefabInterface->CreateEntity(parentEntityId, AZ::Vector3());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }


        // Verify that a valid entity is created.
        AZ::EntityId entityId = createEntityResult.GetValue();
        if (!entityId.IsValid())
        {
            return AZ::Failure(AZStd::string("Invalid id for created entity"));
        }

        AZ_TracePrintf("CreateEntity", "Processing entity id: %s with name: %s\n", entityId.ToString().c_str(), name.c_str());
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        entity->SetName(name);
        entity->Deactivate();
        AddRequiredComponentsToEntity(entityId);
        return createEntityResult;
    }

    void AddRequiredComponentsToEntity(AZ::EntityId entityId)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "Unknown entity %s", entityId.ToString().c_str());
        AzToolsFramework::EditorEntityContextRequestBus::Broadcast(
            &AzToolsFramework::EditorEntityContextRequests::AddRequiredComponents, *entity);
    }

    AZStd::string MakeEntityName(const AZStd::string& rootName, const AZStd::string& type, size_t index)
    {
        const AZStd::string suffix = index == 0 ? AZStd::string() : AZStd::string::format("_%zu", index);
        return AZStd::string::format("%s_%s%s", rootName.c_str(), type.c_str(), suffix.c_str());
    }

    AZStd::optional<Utils::AvailableAsset> GetAssetFromPath(const Utils::UrdfAssetMap& urdfAssetsMapping, const AZStd::string& urdfMeshPath)
    {
        if (!urdfAssetsMapping.contains(urdfMeshPath))
        {
            AZ_Warning("GetAssetFromPath", false, "there is no asset for mesh %s ", urdfMeshPath.c_str());
            return AZStd::optional<Utils::AvailableAsset>();
        }

        return urdfAssetsMapping.at(urdfMeshPath).m_availableAssetInfo;
    }

    AZStd::optional<Utils::AvailableAsset> GetAssetFromPath(const Utils::UrdfAssetMap& urdfAssetsMapping, const std::string& urdfMeshPath)
    {
        return GetAssetFromPath(urdfAssetsMapping, AZStd::string(urdfMeshPath.c_str(), urdfMeshPath.size()));
    }

} // namespace ROS2::PrefabMakerUtils
