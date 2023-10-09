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
        }
        else
        {
            AZ_Warning(Internal::CollidersMakerLoggingTag, false, "Cannot locate wheel material asset");
        }
    }

    void CollidersMaker::AddColliders(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId)
    {
        AZStd::string typeString = "collider";
        const bool isWheelEntity = Utils::IsWheelURDFHeuristics(model, link);
        if (isWheelEntity)
        {
            AZ_Printf(Internal::CollidersMakerLoggingTag, "Due to its name, %s is considered a wheel entity\n", link->Name().c_str());
            if (!m_wheelMaterial.GetId().IsValid())
            {
                FindWheelMaterial();
            }
        }
        const AZ::Data::Asset<Physics::MaterialAsset> materialAsset =
            isWheelEntity ? m_wheelMaterial : AZ::Data::Asset<Physics::MaterialAsset>();
        size_t nameSuffixIndex = 0; // For disambiguation when multiple unnamed colliders are present. The order does not matter here
        for (uint64_t index = 0; index < link->CollisionCount(); index++)
        { // Add colliders (if any) from the collision array
            AddCollider(
                link->CollisionByIndex(index), entityId, PrefabMakerUtils::MakeEntityName(link->Name().c_str(), typeString, nameSuffixIndex), materialAsset);
            nameSuffixIndex++;
        }
    }

    void CollidersMaker::AddCollider(
        const sdf::Collision* collision,
        AZ::EntityId entityId,
        const AZStd::string& generatedName,
        const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset)
    {
        if (!collision)
        { // it is ok not to have collision in a link
            return;
        }
        AZ_Trace(Internal::CollidersMakerLoggingTag, "Processing collisions for entity id:%s\n", entityId.ToString().c_str());

        auto geometry = collision->Geom();
        if (!geometry)
        { // non-empty visual should have a geometry
            AZ_Warning(Internal::CollidersMakerLoggingTag, false, "No Geometry for a collider of entity %s", entityId.ToString().c_str());
            return;
        }

        AddColliderToEntity(collision, entityId, materialAsset);
    }

    void CollidersMaker::AddColliderToEntity(
        const sdf::Collision* collision, AZ::EntityId entityId, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset) const
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "AddColliderToEntity called with invalid entityId");
        auto geometry = collision->Geom();
        bool isPrimitiveShape = geometry->Type() != sdf::GeometryType::MESH;

        Physics::ColliderConfiguration colliderConfig;

        colliderConfig.m_materialSlots.SetMaterialAsset(0, materialAsset);
        colliderConfig.m_position = URDF::TypeConversions::ConvertVector3(collision->RawPose().Pos());
        colliderConfig.m_rotation = URDF::TypeConversions::ConvertQuaternion(collision->RawPose().Rot());
        if (!isPrimitiveShape)
        {
            AZ_Printf(Internal::CollidersMakerLoggingTag, "Adding mesh collider to %s\n", entityId.ToString().c_str());
            auto meshGeometry = geometry->MeshShape();
            AZ_Assert(meshGeometry, "geometry is not meshGeometry");

            auto asset = PrefabMakerUtils::GetAssetFromPath(*m_urdfAssetsMapping, meshGeometry->Uri());
            if (!asset)
            {
                return;
            }

            AZ::Data::AssetId assetId = Utils::GetPhysXMeshProductAssetId(asset->m_sourceGuid);
            if (!assetId.IsValid())
            {
                AZ_Error(
                    Internal::CollidersMakerLoggingTag, false, "Could not find pxmodel for %s", asset->m_sourceAssetGlobalPath.c_str());
                return;
            }
            AZ_Printf(
                Internal::CollidersMakerLoggingTag,
                "Collider %s has assetId %s\n",
                entityId.ToString().c_str(),
                assetId.ToString<AZStd::string>().c_str());

            Physics::PhysicsAssetShapeConfiguration shapeConfiguration;
            auto scale = geometry->MeshShape()->Scale();
            shapeConfiguration.m_assetScale = AZ::Vector3(scale.X(), scale.Y(), scale.Z());
            shapeConfiguration.m_useMaterialsFromAsset = false;
            if (assetId.IsValid())
            {
                auto mesh = AZ::Data::Asset<PhysX::Pipeline::MeshAsset>(assetId, azrtti_typeid<PhysX::Pipeline::MeshAsset>());
                shapeConfiguration.m_asset = mesh;
                entity->CreateComponent<PhysX::EditorMeshColliderComponent>(colliderConfig, shapeConfiguration);
            }
            return;
        }

        AZ_Printf(Internal::CollidersMakerLoggingTag, "URDF/SDF geometry type: %d\n", (int)geometry->Type());
        switch (geometry->Type())
        {
        case sdf::GeometryType::SPHERE:
            {
                auto sphereGeometry = geometry->SphereShape();
                AZ_Assert(sphereGeometry, "geometry is not sphereGeometry");
                const Physics::SphereShapeConfiguration cfg{ static_cast<float>(sphereGeometry->Radius()) };
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, cfg);
            }
            break;
        case sdf::GeometryType::BOX:
            {
                const auto boxGeometry = geometry->BoxShape();
                AZ_Assert(boxGeometry, "geometry is not boxGeometry");
                const Physics::BoxShapeConfiguration cfg{ URDF::TypeConversions::ConvertVector3(boxGeometry->Size()) };
                entity->CreateComponent<PhysX::EditorColliderComponent>(colliderConfig, cfg);
            }
            break;
        case sdf::GeometryType::CYLINDER:
            {
                auto cylinderGeometry = geometry->CylinderShape();
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
                        cylinderGeometry->Length());
                    PhysX::EditorPrimitiveColliderComponentRequestBus::Event(
                        AZ::EntityComponentIdPair(entityId, component->GetId()),
                        &PhysX::EditorPrimitiveColliderComponentRequests::SetCylinderRadius,
                        cylinderGeometry->Radius());
                    PhysX::EditorPrimitiveColliderComponentRequestBus::Event(
                        AZ::EntityComponentIdPair(entityId, component->GetId()),
                        &PhysX::EditorPrimitiveColliderComponentRequests::SetCylinderSubdivisionCount,
                        120);
                    entity->Deactivate();
                }
                else
                {
                    AZ_Warning(Internal::CollidersMakerLoggingTag, false, "The entity was not activated %s", entity->GetName().c_str());
                }
            }
            break;
        default:
            AZ_Warning(Internal::CollidersMakerLoggingTag, false, "Unsupported collider geometry type: %d", (int)geometry->Type());
            break;
        }
    }
} // namespace ROS2
