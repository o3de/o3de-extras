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
#include <LmbrCentral/Shape/BoxShapeComponentBus.h>
#include <LmbrCentral/Shape/CylinderShapeComponentBus.h>
#include <LmbrCentral/Shape/EditorShapeComponentBus.h>
#include <LmbrCentral/Shape/SphereShapeComponentBus.h>

namespace ROS2
{
    VisualsMaker::VisualsMaker(
        const std::map<std::string, urdf::MaterialSharedPtr>& materials, const AZStd::shared_ptr<Utils::UrdfAssetMap>& urdfAssetsMapping)
        : m_urdfAssetsMapping(urdfAssetsMapping)
    {
        AZStd::ranges::for_each(
            materials,
            [&](const auto& p)
            {
                m_materials[AZStd::string(p.first.c_str(), p.first.size())] = p.second;
            });
    }

    AZStd::vector<AZ::EntityId> VisualsMaker::AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId) const
    {
        AZStd::vector<AZ::EntityId> createdEntities;

        const AZStd::string typeString = "visual";
        if (link->visual_array.size() < 1)
        { 
            // For zero visuals, element is used
            auto createdEntity = AddVisual(link->visual, entityId, PrefabMakerUtils::MakeEntityName(link->name.c_str(), typeString));
            if (createdEntity.IsValid())
            {
                createdEntities.emplace_back(createdEntity);
            }
        }
        else
        { 
            // For one or more visuals, an array is used
            size_t nameSuffixIndex = 0; // For disambiguation when multiple unnamed visuals are present. The order does not matter here

            for (auto visual : link->visual_array)
            {
                auto createdEntity = AddVisual(visual, entityId, PrefabMakerUtils::MakeEntityName(link->name.c_str(), typeString, nameSuffixIndex));
                if (createdEntity.IsValid())
                {
                    createdEntities.emplace_back(createdEntity);
                }
                nameSuffixIndex++;
            }
        }
        return createdEntities;
    }

    AZ::EntityId VisualsMaker::AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId, const AZStd::string& generatedName) const
    {
        if (!visual)
        { // It is ok not to have a visual in a link
            return AZ::EntityId();
        }

        if (!visual->geometry)
        { // Non-empty visual should have a geometry. Warn if no geometry present
            AZ_Warning("AddVisual", false, "No Geometry for a visual");
            return AZ::EntityId();
        }

        AZ_Trace("AddVisual", "Processing visual for entity id:%s\n", entityId.ToString().c_str());

        // Use a name generated from the link unless specific name is defined for this visual
        const char* subEntityName = visual->name.empty() ? generatedName.c_str() : visual->name.c_str();
        // Since O3DE does not allow origin for visuals, we need to create a sub-entity and store visual there
        auto createEntityResult = PrefabMakerUtils::CreateEntity(entityId, subEntityName);
        if (!createEntityResult.IsSuccess())
        {
            AZ_Error("AddVisual", false, "Unable to create a sub-entity for visual element %s\n", subEntityName);
            return AZ::EntityId();
        }
        auto visualEntityId = createEntityResult.GetValue();
        AddVisualToEntity(visual, visualEntityId);
        AddMaterialForVisual(visual, visualEntityId);
        return visualEntityId;
    }

    void VisualsMaker::AddVisualToEntity(urdf::VisualSharedPtr visual, AZ::EntityId entityId) const
    {
        // Apply transform as per origin
        PrefabMakerUtils::SetEntityTransformLocal(visual->origin, entityId);

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        auto geometry = visual->geometry;
        switch (geometry->type)
        {
        case urdf::Geometry::SPHERE:
            {
                auto sphereGeometry = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
                AZ_Assert(sphereGeometry, "geometry is not Sphere");
                entity->CreateComponent(LmbrCentral::EditorSphereShapeComponentTypeId);
                entity->Activate();
                LmbrCentral::SphereShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::SphereShapeComponentRequests::SetRadius, sphereGeometry->radius);
                LmbrCentral::EditorShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::EditorShapeComponentRequests::SetVisibleInGame, true);
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::CYLINDER:
            {
                auto cylinderGeometry = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
                AZ_Assert(cylinderGeometry, "geometry is not Cylinder");
                entity->CreateComponent(LmbrCentral::EditorCylinderShapeComponentTypeId);
                entity->Activate();
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetRadius, cylinderGeometry->radius);
                LmbrCentral::CylinderShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::CylinderShapeComponentRequests::SetHeight, cylinderGeometry->length);
                LmbrCentral::EditorShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::EditorShapeComponentRequests::SetVisibleInGame, true);
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::BOX:
            {
                auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(geometry);
                AZ_Assert(boxGeometry, "geometry is not Box");
                entity->CreateComponent(LmbrCentral::EditorBoxShapeComponentTypeId);
                AZ::Vector3 boxDimensions = URDF::TypeConversions::ConvertVector3(boxGeometry->dim);
                entity->Activate();
                LmbrCentral::BoxShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::BoxShapeComponentRequests::SetBoxDimensions, boxDimensions);
                LmbrCentral::EditorShapeComponentRequestsBus::Event(
                    entityId, &LmbrCentral::EditorShapeComponentRequests::SetVisibleInGame, true);
                entity->Deactivate();
            }
            break;
        case urdf::Geometry::MESH:
            {
                auto meshGeometry = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
                AZ_Assert(meshGeometry, "geometry is not Mesh");

                const auto asset = PrefabMakerUtils::GetAssetFromPath(*m_urdfAssetsMapping, meshGeometry->filename);

                if (asset)
                {
                    auto editorMeshComponent = entity->CreateComponent(AZ::Render::EditorMeshComponentTypeId);

                    // Prepare scale
                    AZ::Vector3 scaleVector = URDF::TypeConversions::ConvertVector3(meshGeometry->scale);
                    bool isUniformScale =
                        AZ::IsClose(scaleVector.GetMaxElement(), scaleVector.GetMinElement(), AZ::Constants::FloatEpsilon);
                    if (!isUniformScale)
                    {
                        entity->CreateComponent<AzToolsFramework::Components::EditorNonUniformScaleComponent>();
                    }

                    if (editorMeshComponent)
                    {
                        auto editorBaseComponent = azrtti_cast<AzToolsFramework::Components::EditorComponentBase*>(editorMeshComponent);
                        AZ_Assert(editorBaseComponent, "EditorMeshComponent didn't derive from EditorComponentBase.");

                        AZ::Data::AssetId modelId = Utils::GetModelProductAssetId(asset->m_sourceGuid);
                        AZ_Warning(
                            "AddVisual",
                            modelId.IsValid(),
                            "There is no product asset for %s.",
                            asset->m_sourceAssetRelativePath.c_str());
                        editorBaseComponent->SetPrimaryAsset(modelId);
                    }

                    entity->Activate();

                    // Set scale, uniform or non-uniform
                    if (isUniformScale)
                    {
                        AZ::TransformBus::Event(entityId, &AZ::TransformBus::Events::SetLocalUniformScale, scaleVector.GetX());
                    }
                    else
                    {
                        AZ::NonUniformScaleRequestBus::Event(entityId, &AZ::NonUniformScaleRequests::SetScale, scaleVector);
                    }
                    entity->Deactivate();
                }
            }
            break;
        default:
            AZ_Warning("AddVisual", false, "Unsupported visual geometry type, %d", geometry->type);
            return;
        }
    }

    void VisualsMaker::AddMaterialForVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId) const
    {
        // URDF does not include information from <gazebo> tags with specific materials, diffuse, specular and emissive params
        if (!visual->material || !visual->geometry)
        {
            // Material is optional, and it requires geometry
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        const AZStd::string material_name{ visual->material->name.c_str() };

        // If present in map, take map color definition as priority, otherwise apply local node definition
        const auto materialColorUrdf = m_materials.contains(material_name) ? m_materials.at(material_name)->color : visual->material->color;

        const AZ::Color materialColor = URDF::TypeConversions::ConvertColor(materialColorUrdf);
        bool isPrimitive = visual->geometry->type != urdf::Geometry::MESH;
        if (isPrimitive)
        { // For primitives, set the color in the shape component
            entity->Activate();
            LmbrCentral::EditorShapeComponentRequestsBus::Event(
                entityId, &LmbrCentral::EditorShapeComponentRequests::SetShapeColor, materialColor);
            entity->Deactivate();
            return;
        }

        entity->CreateComponent(AZ::Render::EditorMaterialComponentTypeId);
        AZ_Printf("AddVisual", "Setting color for material %s\n", visual->material->name.c_str());
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
