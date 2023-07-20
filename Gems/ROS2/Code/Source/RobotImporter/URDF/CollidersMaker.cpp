/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CollidersMaker.h"
#include "PrefabMakerUtils.h"
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <PhysX/EditorColliderComponentRequestBus.h>
#include <PhysX/MeshColliderComponentBus.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/DataTypes/Groups/ISceneNodeGroup.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorMeshColliderComponent.h>

namespace ROS2
{
    namespace Internal
    {
        static const char* CollidersMakerLoggingTag = "CollidersMaker";
    } // namespace Internal

    CollidersMaker::CollidersMaker(const AZStd::shared_ptr<Utils::UrdfAssetMap>& urdfAssetsMapping)
        : m_urdfAssetsMapping(urdfAssetsMapping)
    {
    }

    void CollidersMaker::FindWheelMaterial()
    {
        // The `wheel_material.physicsmaterial` is created by Asset Processor from `Materials/wheel_material.physxmaterial`
        // that is provided by this Gem source. The `wheel_material.physicsmaterial` is located in project's cache
        // and that is a critical asset that is automatically loaded.
        const char* physicsMaterialAssetRelPath = "materials/wheel_material.physicsmaterial"; // relative path to cache folder.
        AZ::Data::AssetId assetId;
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
            assetId,
            &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
            physicsMaterialAssetRelPath,
            Physics::MaterialAsset::TYPEINFO_Uuid(),
            false);

        if (assetId.IsValid())
        {
            m_wheelMaterial =
                AZ::Data::Asset<Physics::MaterialAsset>(assetId, Physics::MaterialAsset::TYPEINFO_Uuid(), physicsMaterialAssetRelPath);
            AZ_TracePrintf(Internal::CollidersMakerLoggingTag, "Waiting for asset load\n");
            m_wheelMaterial.BlockUntilLoadComplete();
        }
        else
        {
            AZ_Warning(Internal::CollidersMakerLoggingTag, false, "Cannot load wheel material");
        }
    }

    void CollidersMaker::BuildColliders(urdf::LinkSharedPtr link)
    {
        for (auto collider : link->collision_array)
        {
            BuildCollider(collider);
        }

        if (link->collision_array.empty())
        {
            BuildCollider(link->collision);
        }
    }

    void CollidersMaker::BuildCollider(urdf::CollisionSharedPtr collision)
    {
        if (!collision)
        { // it is ok not to have collision in a link
            return;
        }

        auto geometry = collision->geometry;
        bool isPrimitiveShape = geometry->type != urdf::Geometry::MESH;
        if (!isPrimitiveShape)
        {
            auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
            if (!meshGeometry)
            {
                return;
            }
            const auto asset = PrefabMakerUtils::GetAssetFromPath(*m_urdfAssetsMapping, meshGeometry->filename);
            if (!asset)
            {
                return;
            }
            const AZStd::string& azMeshPath = asset->m_sourceAssetGlobalPath;

            AZStd::shared_ptr<AZ::SceneAPI::Containers::Scene> scene;
            AZ::SceneAPI::Events::SceneSerializationBus::BroadcastResult(
                scene, &AZ::SceneAPI::Events::SceneSerialization::LoadScene, azMeshPath.c_str(), AZ::Uuid::CreateNull(), "");
            if (!scene)
            {
                AZ_Error(
                    Internal::CollidersMakerLoggingTag,
                    false,
                    "Error loading collider. Invalid scene: %s, URDF path: %s",
                    azMeshPath.c_str(),
                    meshGeometry->filename.c_str());
                return;
            }

            AZ::SceneAPI::Containers::SceneManifest& manifest = scene->GetManifest();
            auto valueStorage = manifest.GetValueStorage();
            if (valueStorage.empty())
            {
                AZ_Error(
                    Internal::CollidersMakerLoggingTag, false, "Error loading collider. Invalid value storage: %s", azMeshPath.c_str());
                return;
            }

            auto view = AZ::SceneAPI::Containers::MakeDerivedFilterView<AZ::SceneAPI::DataTypes::ISceneNodeGroup>(valueStorage);
            if (view.empty())
            {
                AZ_Error(Internal::CollidersMakerLoggingTag, false, "Error loading collider. Invalid node views: %s", azMeshPath.c_str());
                return;
            }

            // Select all nodes for both visual and collision nodes
            for (AZ::SceneAPI::DataTypes::ISceneNodeGroup& mg : view)
            {
                AZ::SceneAPI::Utilities::SceneGraphSelector::SelectAll(scene->GetGraph(), mg.GetSceneNodeSelectionList());
            }

            // Update scene with all nodes selected
            AZ::SceneAPI::Events::ProcessingResultCombiner result;
            AZ::SceneAPI::Events::AssetImportRequestBus::BroadcastResult(
                result,
                &AZ::SceneAPI::Events::AssetImportRequest::UpdateManifest,
                *scene,
                AZ::SceneAPI::Events::AssetImportRequest::ManifestAction::Update,
                AZ::SceneAPI::Events::AssetImportRequest::RequestingApplication::Editor);

            if (result.GetResult() != AZ::SceneAPI::Events::ProcessingResult::Success)
            {
                AZ_TracePrintf(Internal::CollidersMakerLoggingTag, "Scene updated\n");
                return;
            }

            auto assetInfoFilePath = AZ::IO::Path{ azMeshPath };
            assetInfoFilePath.Native() += ".assetinfo";
            AZ_Printf(Internal::CollidersMakerLoggingTag, "Saving collider manifest to %s\n", assetInfoFilePath.c_str());
            scene->GetManifest().SaveToFile(assetInfoFilePath.c_str());

            // Set export method to convex mesh
            auto readOutcome = AZ::JsonSerializationUtils::ReadJsonFile(assetInfoFilePath.c_str());
            if (!readOutcome.IsSuccess())
            {
                AZ_Error(
                    Internal::CollidersMakerLoggingTag,
                    false,
                    "Could not read %s with %s",
                    assetInfoFilePath.c_str(),
                    readOutcome.GetError().c_str());
                return;
            }
            rapidjson::Document assetInfoJson = readOutcome.TakeValue();
            auto manifestObject = assetInfoJson.GetObject();
            auto valuesIterator = manifestObject.FindMember("values");
            if (valuesIterator == manifestObject.MemberEnd())
            {
                AZ_Error(
                    Internal::CollidersMakerLoggingTag, false, "Invalid json file: %s (Missing 'values' node)", assetInfoFilePath.c_str());
                return;
            }

            constexpr AZStd::string_view physXMeshGroupType = "{5B03C8E6-8CEE-4DA0-A7FA-CD88689DD45B} MeshGroup";
            auto valuesArray = valuesIterator->value.GetArray();
            for (auto& value : valuesArray)
            {
                auto object = value.GetObject();

                auto physXMeshGroupIterator = object.FindMember("$type");
                if (AZ::StringFunc::Equal(physXMeshGroupIterator->value.GetString(), physXMeshGroupType))
                {
                    value.AddMember(rapidjson::StringRef("export method"), rapidjson::StringRef("1"), assetInfoJson.GetAllocator());
                }
            }

            auto saveOutcome = AZ::JsonSerializationUtils::WriteJsonFile(assetInfoJson, assetInfoFilePath.c_str());
            if (!saveOutcome.IsSuccess())
            {
                AZ_Error(
                    Internal::CollidersMakerLoggingTag,
                    false,
                    "Could not save %s with %s",
                    assetInfoFilePath.c_str(),
                    saveOutcome.GetError().c_str());
                return;
            }
        }
    }

    void CollidersMaker::AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId)
    {
        AZStd::string typeString = "collider";
        const bool isWheelEntity = Utils::IsWheelURDFHeuristics(link);
        if (isWheelEntity)
        {
            AZ_Printf(Internal::CollidersMakerLoggingTag, "Due to its name, %s is considered a wheel entity\n", link->name.c_str());
            if (!m_wheelMaterial.GetId().IsValid())
            {
                FindWheelMaterial();
            }
        }
        const AZ::Data::Asset<Physics::MaterialAsset> materialAsset =
            isWheelEntity ? m_wheelMaterial : AZ::Data::Asset<Physics::MaterialAsset>();
        size_t nameSuffixIndex = 0; // For disambiguation when multiple unnamed colliders are present. The order does not matter here
        for (auto collider : link->collision_array)
        { // Add colliders (if any) from the collision array
            AddCollider(
                collider, entityId, PrefabMakerUtils::MakeEntityName(link->name.c_str(), typeString, nameSuffixIndex), materialAsset);
            nameSuffixIndex++;
        }

        if (nameSuffixIndex == 0)
        { // If there are no colliders in the array, the element member is used instead
            AddCollider(link->collision, entityId, PrefabMakerUtils::MakeEntityName(link->name.c_str(), typeString), materialAsset);
        }
    }

    void CollidersMaker::AddCollider(
        urdf::CollisionSharedPtr collision,
        AZ::EntityId entityId,
        const AZStd::string& generatedName,
        const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset)
    {
        if (!collision)
        { // it is ok not to have collision in a link
            return;
        }
        AZ_TracePrintf(Internal::CollidersMakerLoggingTag, "Processing collisions for entity id:%s\n", entityId.ToString().c_str());

        auto geometry = collision->geometry;
        if (!geometry)
        { // non-empty visual should have a geometry
            AZ_Warning(Internal::CollidersMakerLoggingTag, false, "No Geometry for a collider of entity %s", entityId.ToString().c_str());
            return;
        }

        AddColliderToEntity(collision, entityId, materialAsset);
    }

    void CollidersMaker::AddColliderToEntity(
        urdf::CollisionSharedPtr collision, AZ::EntityId entityId, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset) const
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "AddColliderToEntity called with invalid entityId");
        auto geometry = collision->geometry;
        bool isPrimitiveShape = geometry->type != urdf::Geometry::MESH;

        Physics::ColliderConfiguration colliderConfig;

        colliderConfig.m_materialSlots.SetMaterialAsset(0, materialAsset);
        colliderConfig.m_position = URDF::TypeConversions::ConvertVector3(collision->origin.position);
        colliderConfig.m_rotation = URDF::TypeConversions::ConvertQuaternion(collision->origin.rotation);
        if (!isPrimitiveShape)
        {
            AZ_Printf(Internal::CollidersMakerLoggingTag, "Adding mesh collider to %s\n", entityId.ToString().c_str());
            auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
            AZ_Assert(meshGeometry, "geometry is not meshGeometry");

            auto asset = PrefabMakerUtils::GetAssetFromPath(*m_urdfAssetsMapping, meshGeometry->filename);
            if (!asset)
            {
                return;
            }

            AZStd::string pxmodelPath = Utils::GetPhysXMeshProductAsset(asset->m_sourceGuid);
            if (pxmodelPath.empty())
            {
                AZ_Error(
                    Internal::CollidersMakerLoggingTag, false, "Could not find pxmodel for %s", asset->m_sourceAssetGlobalPath.c_str());
                entity->Deactivate();
                return;
            }
            AZ_Printf(Internal::CollidersMakerLoggingTag, "pxmodelPath  %s\n", pxmodelPath.c_str());
            // Get asset product id (pxmesh)
            AZ::Data::AssetId assetId;
            const AZ::Data::AssetType PhysxMeshAssetType = azrtti_typeid<PhysX::Pipeline::MeshAsset>();
            AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                assetId, &AZ::Data::AssetCatalogRequests::GetAssetIdByPath, pxmodelPath.c_str(), PhysxMeshAssetType, true);
            AZ_Printf(
                Internal::CollidersMakerLoggingTag,
                "Collider %s has assetId %s\n",
                entityId.ToString().c_str(),
                assetId.ToString<AZStd::string>().c_str());

            Physics::PhysicsAssetShapeConfiguration shapeConfiguration;
            shapeConfiguration.m_useMaterialsFromAsset = false;
            if (assetId.IsValid())
            {
                auto mesh = AZ::Data::Asset<PhysX::Pipeline::MeshAsset>(assetId, PhysxMeshAssetType);
                shapeConfiguration.m_asset = mesh;
                entity->CreateComponent<PhysX::EditorMeshColliderComponent>(colliderConfig, shapeConfiguration);
            }
            return;
        }

        AZ_Printf(Internal::CollidersMakerLoggingTag, "URDF geometry type: %d\n", geometry->type);
        switch (geometry->type)
        {
        case urdf::Geometry::SPHERE:
            {
                auto sphereGeometry = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                AZ_Assert(sphereGeometry, "geometry is not sphereGeometry");
                const Physics::SphereShapeConfiguration cfg{ static_cast<float>(sphereGeometry->radius) };
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, cfg);
            }
            break;
        case urdf::Geometry::BOX:
            {
                const auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(geometry);
                AZ_Assert(boxGeometry, "geometry is not boxGeometry");
                const Physics::BoxShapeConfiguration cfg{ URDF::TypeConversions::ConvertVector3(boxGeometry->dim) };
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, cfg);
            }
            break;
        case urdf::Geometry::CYLINDER:
            {
                auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                AZ_Assert(cylinderGeometry, "geometry is not cylinderGeometry");
                Physics::BoxShapeConfiguration cfg;
                auto* component = entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, cfg);
                entity->Activate();
                if (entity->GetState() == AZ::Entity::State::Active)
                {
                    PhysX::EditorPrimitiveColliderComponentRequestBus::Event(
                        AZ::EntityComponentIdPair(entityId, component->GetId()),
                        &PhysX::EditorPrimitiveColliderComponentRequests::SetShapeType,
                        Physics::ShapeType::Cylinder);
                    PhysX::EditorPrimitiveColliderComponentRequestBus::Event(
                        AZ::EntityComponentIdPair(entityId, component->GetId()),
                        &PhysX::EditorPrimitiveColliderComponentRequests::SetCylinderHeight,
                        cylinderGeometry->length);
                    PhysX::EditorPrimitiveColliderComponentRequestBus::Event(
                        AZ::EntityComponentIdPair(entityId, component->GetId()),
                        &PhysX::EditorPrimitiveColliderComponentRequests::SetCylinderRadius,
                        cylinderGeometry->radius);
                    PhysX::EditorPrimitiveColliderComponentRequestBus::Event(
                        AZ::EntityComponentIdPair(entityId, component->GetId()),
                        &PhysX::EditorPrimitiveColliderComponentRequests::SetCylinderSubdivisionCount,
                        120);
                    entity->Deactivate();
                }
                else
                {
                    AZ_Warning(Internal::CollidersMakerLoggingTag, false, "The entity was no activated %s", entity->GetName().c_str());
                }
            }
            break;
        default:
            AZ_Warning(Internal::CollidersMakerLoggingTag, false, "Unsupported collider geometry type: %d", geometry->type);
            break;
        }
    }
} // namespace ROS2
