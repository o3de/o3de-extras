/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/CollidersMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/URDF/TypeConversions.h"
#include "RobotImporter/Utils/RobotImporterUtils.h"
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/DataTypes/Groups/ISceneNodeGroup.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <Source/EditorColliderComponent.h>

namespace ROS2
{
    namespace Internal
    {
        static const char* collidersMakerLoggingTag = "CollidersMaker";
        AZ::IO::Path GetFullURDFMeshPath(AZ::IO::Path modelPath, AZ::IO::Path meshPath)
        {
            modelPath.RemoveFilename();
            AZ::StringFunc::Replace(meshPath.Native(), "package://", "", true, true);
            modelPath /= meshPath;

            return modelPath;
        }

        AZStd::optional<AZ::IO::Path> GetMeshProductPathFromSourcePath(const AZ::IO::Path& sourcePath)
        {
            AZ_TracePrintf(Internal::collidersMakerLoggingTag, "GetMeshProductPathFromSourcePath: %s", sourcePath.c_str());
            AZ::Data::AssetInfo assetInfo;

            AZStd::string watchDir;
            bool assetFound = false;
            AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
                assetFound,
                &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
                sourcePath.c_str(),
                assetInfo,
                watchDir);

            if (!assetFound)
            {
                AZ_Error(Internal::collidersMakerLoggingTag, false, "Could not find asset %s", sourcePath.c_str());
                return {};
            }

            AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;

            bool productsFound = false;
            AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
                productsFound,
                &AzToolsFramework::AssetSystem::AssetSystemRequest::GetAssetsProducedBySourceUUID,
                assetInfo.m_assetId.m_guid,
                productsAssetInfo);

            if (!productsFound)
            {
                AZ_Error(Internal::collidersMakerLoggingTag, false, "Could not find products for asset %s", sourcePath.c_str());
                return {};
            }

            AZStd::vector<AZ::IO::Path> productsPaths;
            AZStd::transform(
                productsAssetInfo.cbegin(),
                productsAssetInfo.cend(),
                AZStd::back_inserter(productsPaths),
                [](const AZ::Data::AssetInfo& assetInfo)
                {
                    AZStd::string assetPath;
                    AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                        assetPath, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetPathById, assetInfo.m_assetId);
                    return AZ::IO::Path(assetPath);
                });

            AZStd::vector<AZ::IO::Path> pxMeshesPaths;
            AZStd::remove_copy_if(
                productsPaths.cbegin(),
                productsPaths.cend(),
                AZStd::back_inserter(pxMeshesPaths),
                [](const AZ::IO::Path& path)
                {
                    return !(path.Extension() == ".pxmesh");
                });

            if (pxMeshesPaths.empty())
            {
                return {};
            }

            AZ_Assert(pxMeshesPaths.size() == 1, "Currently only one pxmesh for each model source is supported by robot importer");

            return pxMeshesPaths.front();
        }
    } // namespace Internal

    CollidersMaker::CollidersMaker(AZStd::string modelPath)
        : m_modelPath(AZStd::move(modelPath))
        , m_stopBuildFlag(false)
    {
        FindWheelMaterial();
    }

    CollidersMaker::~CollidersMaker()
    {
        m_stopBuildFlag = true;
        m_buildThread.join();
    };

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
            AZ_TracePrintf(Internal::collidersMakerLoggingTag, "Wait for loading asset\n");
            m_wheelMaterial.BlockUntilLoadComplete();
        }
        else
        {
            AZ_Warning(Internal::collidersMakerLoggingTag, false, "Cannot load wheel material");
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
            auto azMeshPath = Internal::GetFullURDFMeshPath(AZ::IO::Path(m_modelPath), AZ::IO::Path(meshGeometry->filename.c_str()));

            AZStd::shared_ptr<AZ::SceneAPI::Containers::Scene> scene;
            AZ::SceneAPI::Events::SceneSerializationBus::BroadcastResult(
                scene, &AZ::SceneAPI::Events::SceneSerialization::LoadScene, azMeshPath.c_str(), AZ::Uuid::CreateNull());
            if (!scene)
            {
                AZ_Error(
                    Internal::collidersMakerLoggingTag,
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
                    Internal::collidersMakerLoggingTag, false, "Error loading collider. Invalid value storage: %s", azMeshPath.c_str());
                return;
            }

            auto view = AZ::SceneAPI::Containers::MakeDerivedFilterView<AZ::SceneAPI::DataTypes::ISceneNodeGroup>(valueStorage);
            if (view.empty())
            {
                AZ_Error(Internal::collidersMakerLoggingTag, false, "Error loading collider. Invalid node views: %s", azMeshPath.c_str());
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
                AZ_TracePrintf(Internal::collidersMakerLoggingTag, "Scene updated\n");
                return;
            }

            auto assetInfoFilePath = azMeshPath;
            assetInfoFilePath.Native() += ".assetinfo";
            AZ_Printf(Internal::collidersMakerLoggingTag, "Saving collider manifest to %s", assetInfoFilePath.c_str());
            scene->GetManifest().SaveToFile(assetInfoFilePath.c_str());

            bool assetFound = false;
            AZ::Data::AssetInfo assetInfo;
            AZStd::string watchDir;
            AzToolsFramework::AssetSystemRequestBus::BroadcastResult(
                assetFound,
                &AzToolsFramework::AssetSystem::AssetSystemRequest::GetSourceInfoBySourcePath,
                azMeshPath.c_str(),
                assetInfo,
                watchDir);

            // Set export method to convex mesh
            auto readOutcome = AZ::JsonSerializationUtils::ReadJsonFile(assetInfoFilePath.c_str());
            if (!readOutcome.IsSuccess())
            {
                AZ_Error(
                    Internal::collidersMakerLoggingTag,
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
                    Internal::collidersMakerLoggingTag, false, "Invalid json file: %s (Missing 'values' node)", assetInfoFilePath.c_str());
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
                    Internal::collidersMakerLoggingTag,
                    false,
                    "Could not save %s with %s",
                    assetInfoFilePath.c_str(),
                    saveOutcome.GetError().c_str());
                return;
            }

            // Add asset to expected assets list
            if (assetFound)
            {
                AZStd::lock_guard lock{ m_buildMutex };
                m_meshesToBuild.push_back(AZ::IO::Path(assetInfo.m_relativePath));
            }
        }
    }

    void CollidersMaker::AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId)
    {
        AZStd::string typeString = "collider";
        const bool isWheelEntity = Utils::IsWheelURDFHeuristics(link);
        if (isWheelEntity)
        {
            AZ_Printf(Internal::collidersMakerLoggingTag, "%s is wheel", link->name.c_str());
        }
        const AZ::Data::Asset<Physics::MaterialAsset> materialAsset =
            isWheelEntity ? m_wheelMaterial : AZ::Data::Asset<Physics::MaterialAsset>();
        size_t nameSuffixIndex = 0; // For disambiguation when multiple unnamed colliders are present. The order does not matter here
        for (auto collider : link->collision_array)
        { // one or more colliders - the array is used
            AddCollider(
                collider, entityId, PrefabMakerUtils::MakeEntityName(link->name.c_str(), typeString, nameSuffixIndex), materialAsset);
            nameSuffixIndex++;
        }

        if (nameSuffixIndex == 0)
        { // no colliders in the array - zero or one in total, the element member is used instead
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
        AZ_TracePrintf(Internal::collidersMakerLoggingTag, "Processing collisions for entity id:%s\n", entityId.ToString().c_str());

        auto geometry = collision->geometry;
        if (!geometry)
        { // non-empty visual should have a geometry
            AZ_Warning(Internal::collidersMakerLoggingTag, false, "No Geometry for a collider");
            return;
        }

        AddColliderToEntity(collision, entityId, materialAsset);
    }

    void CollidersMaker::AddColliderToEntity(
        urdf::CollisionSharedPtr collision, AZ::EntityId entityId, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset)
    {
        // TODO - we are unable to set collider origin. Sub-entities don't work since they would need to parent visuals etc.
        // TODO - solution: once Collider Component supports Cylinder Shape, switch to it from Shape Collider Component.

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
            // TODO move setting mesh with ebus here - othervise material is not assigned
            Physics::PhysicsAssetShapeConfiguration shapeConfiguration;
            shapeConfiguration.m_useMaterialsFromAsset = false;
            entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, shapeConfiguration);
            entity->Activate();

            AZ_Printf(Internal::collidersMakerLoggingTag, "Adding mesh collider to %s\n", entityId.ToString().c_str());
            auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
            AZ_Assert(meshGeometry, "geometry is not meshGeometry");
            auto azMeshPath = Internal::GetFullURDFMeshPath(AZ::IO::Path(m_modelPath), AZ::IO::Path(meshGeometry->filename.c_str()));
            AZStd::optional<AZ::IO::Path> pxmodelPath = Internal::GetMeshProductPathFromSourcePath(azMeshPath);
            if (!pxmodelPath)
            {
                AZ_Error(Internal::collidersMakerLoggingTag, false, "Could not find pxmodel for %s", azMeshPath.c_str());
                return;
            }

            // Get asset product id (pxmesh)
            AZ::Data::AssetId assetId;
            AZ::Data::AssetType assetType = AZ::AzTypeInfo<PhysX::Pipeline::MeshAsset>::Uuid();
            AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                assetId, &AZ::Data::AssetCatalogRequests::GetAssetIdByPath, pxmodelPath->c_str(), assetType, false);
            // Insert pxmesh into the collider component
            PhysX::MeshColliderComponentRequestsBus::Event(entityId, &PhysX::MeshColliderComponentRequests::SetMeshAsset, assetId);
            entity->Deactivate();

            return;
        }

        AZ_Printf(Internal::collidersMakerLoggingTag, "URDF geometry type : %d\n", geometry->type);
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
                // TODO HACK Underlying API of O3DE  does not have Physic::CylinderShapeConfiguration implementation
                Physics::BoxShapeConfiguration cfg;
                auto* component = entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, cfg);
                entity->Activate();
                PhysX::EditorColliderComponentRequestBus::Event(
                    AZ::EntityComponentIdPair(entityId, component->GetId()),
                    &PhysX::EditorColliderComponentRequests::SetShapeType,
                    Physics::ShapeType::Cylinder);
                PhysX::EditorColliderComponentRequestBus::Event(
                    AZ::EntityComponentIdPair(entityId, component->GetId()),
                    &PhysX::EditorColliderComponentRequests::SetCylinderHeight,
                    cylinderGeometry->length);
                PhysX::EditorColliderComponentRequestBus::Event(
                    AZ::EntityComponentIdPair(entityId, component->GetId()),
                    &PhysX::EditorColliderComponentRequests::SetCylinderRadius,
                    cylinderGeometry->radius);
                entity->Deactivate();
            }
            break;
        default:
            AZ_Warning(Internal::collidersMakerLoggingTag, false, "Unsupported collider geometry type, %d", geometry->type);
            break;
        }
    }

    void CollidersMaker::ProcessMeshes(BuildReadyCallback notifyBuildReadyCb)
    {
        m_buildThread = AZStd::thread(
            [this, notifyBuildReadyCb]()
            {
                AZ_Printf(Internal::collidersMakerLoggingTag, "Waiting for URDF assets\n");

                while (!m_meshesToBuild.empty() && !m_stopBuildFlag)
                {
                    {
                        AZStd::lock_guard lock{ m_buildMutex };
                        auto eraseFoundMesh = [](const AZ::IO::Path& meshPath)
                        {
                            return Internal::GetMeshProductPathFromSourcePath(meshPath).has_value();
                        };
                        AZStd::erase_if(m_meshesToBuild, AZStd::move(eraseFoundMesh));
                    }
                    if (!m_meshesToBuild.empty())
                    {
                        AZStd::this_thread::sleep_for(AZStd::chrono::milliseconds(50));
                    }
                }

                AZ_Printf(Internal::collidersMakerLoggingTag, "All URDF assets ready!\n");
                // Notify the caller that we can continue with constructing the prefab.
                notifyBuildReadyCb();
            });
    }
} // namespace ROS2
