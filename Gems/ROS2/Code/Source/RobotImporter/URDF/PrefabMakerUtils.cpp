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
#include <AzToolsFramework/Entity/EntityUtilityComponent.h>
#include <AzToolsFramework/Prefab/PrefabFocusPublicInterface.h>
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

    void SetEntityTransformLocal(const gz::math::Pose3d& origin, AZ::EntityId entityId)
    {
        gz::math::Vector3 urdfPosition = origin.Pos();
        gz::math::Quaternion urdfRotation = origin.Rot();
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
        // Create an entity with the appropriate Editor components, but in a not-yet-activated state.
        AZ::EntityId entityId;
        AzToolsFramework::EntityUtilityBus::BroadcastResult(
            entityId, &AzToolsFramework::EntityUtilityBus::Events::CreateEditorReadyEntity, name);

        if (entityId.IsValid() == false)
        {
            return AZ::Failure(AZStd::string("Invalid id for created entity"));
        }

        AZ_Trace("CreateEntity", "Processing entity id: %s with name: %s\n", entityId.ToString().c_str(), name.c_str());

        // If the parent is invalid, parent to the container of the currently focused prefab if one exists.
        if (!parentEntityId.IsValid())
        {
            AzFramework::EntityContextId editorEntityContextId = AzToolsFramework::GetEntityContextId();

            auto prefabFocusPublicInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabFocusPublicInterface>::Get();
            if (prefabFocusPublicInterface)
            {
                parentEntityId = prefabFocusPublicInterface->GetFocusedPrefabContainerEntityId(editorEntityContextId);
            }
        }

        // Default the entity world transform to be the same as the parent entity world transform
        // Calling SetEntityParent would have the transform be at world origin
        SetEntityParentRelative(entityId, parentEntityId);

        return entityId;
    }

    static void SetEntityParentInternal(AZ::EntityId entityId, AZ::EntityId parentEntityId, bool useLocalTransform)
    {
        auto* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "Unknown entity %s", entityId.ToString().c_str());
        AZ_Assert(
            (entity->GetState() == AZ::Entity::State::Constructed) || (entity->GetState() == AZ::Entity::State::Init),
            "Entity must be inactive when getting reparented.");

        if (auto* transformComponent = entity->FindComponent<AzToolsFramework::Components::TransformComponent>(); transformComponent)
        {
            if (useLocalTransform)
            {
                transformComponent->SetParentRelative(parentEntityId);
            }
            else
            {
                transformComponent->SetParent(parentEntityId);
            }
        }
    }

    void SetEntityParent(AZ::EntityId entityId, AZ::EntityId parentEntityId)
    {
        constexpr bool useLocalTransform = false;
        return SetEntityParentInternal(entityId, parentEntityId, useLocalTransform);
    }

    void SetEntityParentRelative(AZ::EntityId entityId, AZ::EntityId parentEntityId)
    {
        constexpr bool useLocalTransform = true;
        return SetEntityParentInternal(entityId, parentEntityId, useLocalTransform);
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
