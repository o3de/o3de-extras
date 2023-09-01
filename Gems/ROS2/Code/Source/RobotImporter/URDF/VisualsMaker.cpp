/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/VisualsMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/Utils/TypeConversions.h"

#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentConstants.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentConstants.h>
#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/EditorNonUniformScaleComponent.h>

namespace ROS2
{
    VisualsMaker::VisualsMaker() = default;
    VisualsMaker::VisualsMaker(MaterialNameMap materials, const AZStd::shared_ptr<Utils::UrdfAssetMap>& urdfAssetsMapping)
        : m_materials(AZStd::move(materials))
        , m_urdfAssetsMapping(urdfAssetsMapping)
    {
    }

    AZStd::vector<AZ::EntityId> VisualsMaker::AddVisuals(const sdf::Link* link, AZ::EntityId entityId) const
    {
        AZStd::vector<AZ::EntityId> createdEntities;

        const AZStd::string typeString = "visual";
        if (link->VisualCount() < 1)
        {
            // For zero visuals, element is used
            auto createdEntity = AddVisual(nullptr, entityId, PrefabMakerUtils::MakeEntityName(link->Name().c_str(), typeString));
            if (createdEntity.IsValid())
            {
                createdEntities.emplace_back(createdEntity);
            }
        }
        else
        {
            // For one or more visuals, an array is used
            size_t nameSuffixIndex = 0; // For disambiguation when multiple unnamed visuals are present. The order does not matter here

            for (uint64_t index = 0; index < link->VisualCount(); index++)
            {
                auto createdEntity = AddVisual(
                    link->VisualByIndex(index),
                    entityId,
                    PrefabMakerUtils::MakeEntityName(link->Name().c_str(), typeString, nameSuffixIndex));
                if (createdEntity.IsValid())
                {
                    createdEntities.emplace_back(createdEntity);
                }
                nameSuffixIndex++;
            }
        }
        return createdEntities;
    }

    AZ::EntityId VisualsMaker::AddVisual(const sdf::Visual* visual, AZ::EntityId entityId, const AZStd::string& generatedName) const
    {
        if (!visual)
        { // It is ok not to have a visual in a link
            return AZ::EntityId();
        }

        if (!visual->Geom())
        { // Non-empty visual should have a geometry. Warn if no geometry present
            AZ_Warning("AddVisual", false, "No Geometry for a visual");
            return AZ::EntityId();
        }

        AZ_Trace("AddVisual", "Processing visual for entity id:%s\n", entityId.ToString().c_str());

        // Use a name generated from the link unless specific name is defined for this visual
        AZStd::string subEntityName = visual->Name().empty() ? generatedName.c_str() : visual->Name().c_str();
        // Since O3DE does not allow origin for visuals, we need to create a sub-entity and store visual there
        auto createEntityResult = PrefabMakerUtils::CreateEntity(entityId, subEntityName.c_str());
        if (!createEntityResult.IsSuccess())
        {
            AZ_Error("AddVisual", false, "Unable to create a sub-entity for visual element %s\n", subEntityName.c_str());
            return AZ::EntityId();
        }
        auto visualEntityId = createEntityResult.GetValue();
        AddVisualToEntity(visual, visualEntityId);
        AddMaterialForVisual(visual, visualEntityId);
        return visualEntityId;
    }

    void VisualsMaker::AddVisualToEntity(const sdf::Visual* visual, AZ::EntityId entityId) const
    {
        // Apply transform as per origin
        PrefabMakerUtils::SetEntityTransformLocal(visual->RawPose(), entityId);

        auto geometry = visual->Geom();
        switch (geometry->Type())
        {
        case sdf::GeometryType::SPHERE:
            {
                auto sphereGeometry = geometry->SphereShape();
                AZ_Assert(sphereGeometry, "geometry is not Sphere");
                // Convert radius to diameter: the `_sphere_1x1.fbx.azmodel` model has a diameter of 1
                const AZ::Vector3 sphereDimensions(sphereGeometry->Radius() * 2);

                // The `_sphere_1x1.fbx.azmodel` is created by Asset Processor based on O3DE `PrimitiveAssets` Gem source.
                AZ::Data::AssetId assetId;
                const char* sphereAssetRelPath = "objects/_primitives/_sphere_1x1.fbx.azmodel"; // relative path to cache folder.
                AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                    assetId,
                    &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
                    sphereAssetRelPath,
                    AZ::Data::s_invalidAssetType,
                    false);
                AZ_Warning("AddVisual", assetId.IsValid(), "There is no product asset for %s.", sphereAssetRelPath);

                AddVisualAssetToEntity(entityId, assetId, sphereDimensions);
            }
            break;
        case sdf::GeometryType::CYLINDER:
            {
                auto cylinderGeometry = geometry->CylinderShape();
                AZ_Assert(cylinderGeometry, "geometry is not Cylinder");
                // Convert radius to diameter: the `_cylinder_1x1.fbx.azmodel` model has a diameter of 1
                const AZ::Vector3 cylinderDimensions(
                    cylinderGeometry->Radius() * 2, cylinderGeometry->Radius() * 2, cylinderGeometry->Length());

                // The `_cylinder_1x1.fbx.azmodel` is created by Asset Processor based on O3DE `PrimitiveAssets` Gem source.
                AZ::Data::AssetId assetId;
                const char* cylinderAssetRelPath = "objects/_primitives/_cylinder_1x1.fbx.azmodel"; // relative path to cache folder.
                AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                    assetId,
                    &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
                    cylinderAssetRelPath,
                    AZ::Data::s_invalidAssetType,
                    false);
                AZ_Warning("AddVisual", assetId.IsValid(), "There is no product asset for %s.", cylinderAssetRelPath);

                AddVisualAssetToEntity(entityId, assetId, cylinderDimensions);
            }
            break;
        case sdf::GeometryType::BOX:
            {
                auto boxGeometry = geometry->BoxShape();
                AZ_Assert(boxGeometry, "geometry is not Box");
                const AZ::Vector3 boxDimensions = URDF::TypeConversions::ConvertVector3(boxGeometry->Size());

                // The `_box_1x1.fbx.azmodel` is created by Asset Processor based on O3DE `PrimitiveAssets` Gem source.
                AZ::Data::AssetId assetId;
                const char* boxAssetRelPath = "objects/_primitives/_box_1x1.fbx.azmodel"; // relative path to cache folder.
                AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                    assetId,
                    &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
                    boxAssetRelPath,
                    AZ::Data::s_invalidAssetType,
                    false);
                AZ_Warning("AddVisual", assetId.IsValid(), "There is no product asset for %s.", boxAssetRelPath);

                AddVisualAssetToEntity(entityId, assetId, boxDimensions);
            }
            break;
        case sdf::GeometryType::MESH:
            {
                auto meshGeometry = geometry->MeshShape();
                AZ_Assert(meshGeometry, "geometry is not Mesh");
                const AZ::Vector3 scaleVector = URDF::TypeConversions::ConvertVector3(meshGeometry->Scale());

                const auto asset = PrefabMakerUtils::GetAssetFromPath(*m_urdfAssetsMapping, AZStd::string(meshGeometry->Uri().c_str()));
                AZ::Data::AssetId assetId = Utils::GetModelProductAssetId(asset->m_sourceGuid);
                AZ_Warning("AddVisual", assetId.IsValid(), "There is no product asset for %s.", asset->m_sourceAssetRelativePath.c_str());

                AddVisualAssetToEntity(entityId, assetId, scaleVector);
            }
            break;
        default:
            AZ_Warning("AddVisual", false, "Unsupported visual geometry type, %d", (int)geometry->Type());
            return;
        }
    }

    void VisualsMaker::AddVisualAssetToEntity(AZ::EntityId entityId, const AZ::Data::AssetId& assetId, const AZ::Vector3& scale) const
    {
        if (!assetId.IsValid())
        {
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto editorMeshComponent = entity->CreateComponent(AZ::Render::EditorMeshComponentTypeId);

        // Prepare scale
        bool isUniformScale = AZ::IsClose(scale.GetMaxElement(), scale.GetMinElement(), AZ::Constants::FloatEpsilon);
        if (!isUniformScale)
        {
            entity->CreateComponent<AzToolsFramework::Components::EditorNonUniformScaleComponent>();
        }

        if (editorMeshComponent)
        {
            auto editorBaseComponent = azrtti_cast<AzToolsFramework::Components::EditorComponentBase*>(editorMeshComponent);
            AZ_Assert(editorBaseComponent, "EditorMeshComponent didn't derive from EditorComponentBase.");
            editorBaseComponent->SetPrimaryAsset(assetId);
        }

        entity->Activate();

        // Set scale, uniform or non-uniform
        if (isUniformScale)
        {
            AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalUniformScale, scale.GetX());
        }
        else
        {
            AZ::NonUniformScaleRequestBus::Event(entityId, &AZ::NonUniformScaleRequests::SetScale, scale);
        }
        entity->Deactivate();
    }

    void VisualsMaker::AddMaterialForVisual(const sdf::Visual* visual, AZ::EntityId entityId) const
    {
        // URDF does not include information from <gazebo> tags with specific materials, diffuse, specular and emissive params
        if (!visual->Material())
        {
            // Material is optional, and it requires geometry
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        // As a Material doesn't have a name and there can only be 1 material per <visual> tag,
        // the Visual Name is used for the material
        const std::string materialName{ visual->Name() };
        const AZStd::string azMaterialName{ materialName.c_str(), materialName.size() };

        // If present in map, take map color definition as priority, otherwise apply local node definition
        const auto materialColorUrdf =
            m_materials.contains(azMaterialName) ? m_materials.at(azMaterialName)->Diffuse() : visual->Material()->Diffuse();
        const AZ::Color materialColor = URDF::TypeConversions::ConvertColor(materialColorUrdf);

        entity->CreateComponent(AZ::Render::EditorMaterialComponentTypeId);
        AZ_Trace("AddVisual", "Setting color for material %s\n", azMaterialName.c_str());
        entity->Activate();
        AZ::Render::MaterialComponentRequestBus::Event(
            entityId,
            &AZ::Render::MaterialComponentRequestBus::Events::SetPropertyValue,
            AZ::Render::DefaultMaterialAssignmentId,
            "settings.color",
            AZStd::any(materialColor));
        entity->Deactivate();
    }
} // namespace ROS2
