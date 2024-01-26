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

#include <Atom/RPI.Reflect/Material/MaterialAsset.h>
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentConfig.h>
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentConstants.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentConstants.h>
#include <AzCore/Component/NonUniformScaleBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/EditorNonUniformScaleComponent.h>

#include <sdf/Material.hh>
#include <sdf/Pbr.hh>

namespace ROS2
{
    VisualsMaker::VisualsMaker() = default;
    VisualsMaker::VisualsMaker(const AZStd::shared_ptr<Utils::UrdfAssetMap>& urdfAssetsMapping)
        : m_urdfAssetsMapping(urdfAssetsMapping)
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
        auto createEntityResult = PrefabMakerUtils::CreateEntity(entityId, subEntityName);
        if (!createEntityResult.IsSuccess())
        {
            AZ_Error("AddVisual", false, "Unable to create a sub-entity for visual element %s\n", subEntityName.c_str());
            return AZ::EntityId();
        }
        auto visualEntityId = createEntityResult.GetValue();
        auto visualAssetId = AddVisualToEntity(visual, visualEntityId);
        AddMaterialForVisual(visual, visualEntityId, visualAssetId);
        return visualEntityId;
    }

    AZ::Data::AssetId VisualsMaker::AddVisualToEntity(const sdf::Visual* visual, AZ::EntityId entityId) const
    {
        // Asset ID for the asset added to the visual entity, if any.
        AZ::Data::AssetId assetId;

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
                const AZ::Vector3 sphereDimensions(sphereGeometry->Radius() * 2.0f);

                // The `_sphere_1x1.fbx.azmodel` is created by Asset Processor based on O3DE `PrimitiveAssets` Gem source.
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
                    cylinderGeometry->Radius() * 2.0f, cylinderGeometry->Radius() * 2.0f, cylinderGeometry->Length());

                // The `_cylinder_1x1.fbx.azmodel` is created by Asset Processor based on O3DE `PrimitiveAssets` Gem source.
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
                AZ_Warning("AddVisual", asset, "There is no source asset for %s.", meshGeometry->Uri().c_str());

                if (asset)
                {
                    assetId = Utils::GetModelProductAssetId(asset->m_sourceGuid);
                    AZ_Warning("AddVisual", assetId.IsValid(), "There is no product asset for %s.", asset->m_sourceAssetRelativePath.c_str());
                }

                AddVisualAssetToEntity(entityId, assetId, scaleVector);
            }
            break;
        default:
            AZ_Warning("AddVisual", false, "Unsupported visual geometry type, %d", (int)geometry->Type());
            break;
        }

        return assetId;
    }

    void VisualsMaker::AddVisualAssetToEntity(AZ::EntityId entityId, const AZ::Data::AssetId& assetId, const AZ::Vector3& scale) const
    {
        if (!assetId.IsValid())
        {
            return;
        }

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        bool isUniformScale = AZ::IsClose(scale.GetMaxElement(), scale.GetMinElement(), AZ::Constants::FloatEpsilon);
        if (isUniformScale)
        {
            auto* transformComponent = entity->FindComponent(AZ::EditorTransformComponentTypeId);
            AZ_Assert(transformComponent, "Entity doesn't have a transform component.");
            auto* transformInterface = azrtti_cast<AZ::TransformInterface*>(transformComponent);
            AZ_Assert(transformInterface, "Found component has no transformInterface");
            transformInterface->SetLocalUniformScale(scale.GetX());
        }
        else
        {
            auto component = entity->CreateComponent<AzToolsFramework::Components::EditorNonUniformScaleComponent>();
            AZ_Assert(component, "EditorNonUniformScaleComponent was not created");
            component->SetScale(scale);
        }

        auto editorMeshComponent = entity->CreateComponent(AZ::Render::EditorMeshComponentTypeId);
        if (editorMeshComponent)
        {
            auto editorBaseComponent = azrtti_cast<AzToolsFramework::Components::EditorComponentBase*>(editorMeshComponent);
            AZ_Assert(editorBaseComponent, "EditorMeshComponent didn't derive from EditorComponentBase.");
            editorBaseComponent->SetPrimaryAsset(assetId);
        }
    }

    static void OverrideScriptMaterial(const sdf::Material* material, AZ::Render::MaterialAssignmentMap& overrides)
    {
        AZStd::string materialName(material->ScriptName().c_str(), material->ScriptName().size());
        if (materialName.empty())
        {
            return;
        }

        // Make sure the material name is lowercased before checking the path in the Asset Cache
        AZStd::to_lower(materialName);
        // If a material has a <script> element we'll treat the name as a path and name to an O3DE material.
        // For example, "Gazebo/Wood" will look for a product material in "<cache>/gazebo/wood.azmaterial"
        AZ::IO::Path materialProductPath(materialName);
        materialProductPath.ReplaceExtension(".azmaterial");

        // Try getting an asset ID for the given name.
        constexpr bool AutoGenerateId = false;
        AZ::Data::AssetId assetId;
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
            assetId,
            &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
            materialProductPath.String().c_str(),
            azrtti_typeid<AZ::RPI::MaterialAsset>(),
            AutoGenerateId);

        // No asset was found, we can't convert the material script.
        if (!assetId.IsValid())
        {
            AZ_Warning("AddMaterial", false, "Failed to find product material for %s", materialProductPath.c_str());
            return;
        }

        // The asset was found, so replace all the material assets in the given material assignment map.
        AZ::Data::Asset<AZ::RPI::MaterialAsset> materialAsset(assetId, azrtti_typeid<AZ::RPI::MaterialAsset>());
        for (auto& [id, override] : overrides)
        {
            if (id == AZ::Render::DefaultMaterialAssignmentId)
            {
                override.m_defaultMaterialAsset = materialAsset;
            }
            else
            {
                override.m_materialAsset = materialAsset;
            }
        }

        AZ_Info("AddMaterial", "Added product material %s\n", materialProductPath.c_str());
    }

    static void OverrideMaterialPbrSettings(
        const sdf::Material* material,
        const AZStd::shared_ptr<Utils::UrdfAssetMap>& assetMapping,
        AZ::Render::MaterialAssignmentMap& overrides)
    {
        if (auto pbr = material->PbrMaterial(); pbr)
        {
            // Start by trying to get the Metal workflow, since this is the workflow that O3DE uses.
            auto pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::METAL);

            if (!pbrWorkflow)
            {
                // If the Metal workflow doesn't exist, try to get the Specular workflow.
                // Even though O3DE uses a Metal workflow, the vast majority of the Specular workflow data can still be used.
                // It's only the specular/glossiness maps that won't be converted.
                pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
                if (!pbrWorkflow)
                {
                    AZ_Error("AddMaterial", false, "Material has a PBR definition, but it is neither a Metal nor a Specular workflow. Cannot convert.");
                    return;
                }
            }

            for (auto& [id, materialAssignment] : overrides)
            {
                auto GetImageAssetIdFromPath = [&assetMapping](const std::string& uri) -> AZ::Data::AssetId
                {
                    AZ::Data::AssetId assetId;
                    const auto asset = PrefabMakerUtils::GetAssetFromPath(*assetMapping, uri);
                    AZ_Warning("AddVisual", asset, "There is no source image asset for %s.", uri.c_str());

                    if (asset)
                    {
                        assetId = Utils::GetImageProductAssetId(asset->m_sourceGuid);
                        AZ_Warning("AddVisual", assetId.IsValid(), "There is no product image asset for %s.", asset->m_sourceAssetRelativePath.c_str());
                    }
                    return assetId;
                };

                if (auto texture = pbrWorkflow->AlbedoMap(); !texture.empty())
                {
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("baseColor.textureMap"), AZStd::any(GetImageAssetIdFromPath(texture)));
                }

                if (auto texture = pbrWorkflow->NormalMap(); !texture.empty())
                {
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("normal.textureMap"), AZStd::any(GetImageAssetIdFromPath(texture)));
                }

                if (auto texture = pbrWorkflow->AmbientOcclusionMap(); !texture.empty())
                {
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("occlusion.diffuseTextureMap"), AZStd::any(GetImageAssetIdFromPath(texture)));
                }

                if (auto texture = pbrWorkflow->EmissiveMap(); !texture.empty())
                {
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("emissive.enable"), AZStd::any(true));
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("emissive.textureMap"), AZStd::any(GetImageAssetIdFromPath(texture)));
                }

                if (pbrWorkflow->Type() == sdf::PbrWorkflowType::METAL)
                {
                    if (auto texture = pbrWorkflow->RoughnessMap(); !texture.empty())
                    {
                        materialAssignment.m_propertyOverrides.emplace(AZ::Name("roughness.textureMap"), AZStd::any(GetImageAssetIdFromPath(texture)));
                    }

                    if (auto texture = pbrWorkflow->MetalnessMap(); !texture.empty())
                    {
                        materialAssignment.m_propertyOverrides.emplace(AZ::Name("metallic.textureMap"), AZStd::any(GetImageAssetIdFromPath(texture)));
                    }

                    if (pbrWorkflow->Element()->HasElement("roughness"))
                    {
                        materialAssignment.m_propertyOverrides.emplace(AZ::Name("roughness.factor"), AZStd::any(static_cast<float>(pbrWorkflow->Roughness())));
                    }

                    if (pbrWorkflow->Element()->HasElement("metalness"))
                    {
                        materialAssignment.m_propertyOverrides.emplace(AZ::Name("metallic.factor"), AZStd::any(static_cast<float>(pbrWorkflow->Metalness())));
                    }
                }
                else
                {
                    AZ_Warning("AddMaterial", pbrWorkflow->GlossinessMap().empty(), 
                        "PBR material has a Glossiness map (%s), which is a Specular PBR workflow, not a Metal PBR workflow. It will not be converted.", pbrWorkflow->GlossinessMap().c_str());
                    AZ_Warning("AddMaterial", pbrWorkflow->SpecularMap().empty(), 
                        "PBR material has a Specular map (%s), which is a Specular PBR workflow, not a Metal PBR workflow. It will not be converted.", pbrWorkflow->SpecularMap().c_str());
                }
            }
        }
    }

    static void OverrideMaterialBaseColor(const sdf::Material* material, AZ::Render::MaterialAssignmentMap& overrides)
    {
        // Base Color: Try to use the diffuse color if the material has one, or fall back to using the ambient color as a backup option.
        if (material->Element()->HasElement("diffuse") || material->Element()->HasElement("ambient"))
        {
            // Get the material's diffuse color as the preferred option for the PBR material base color.
            // If a diffuse color didn't exist but ambient does, try to use that instead as the base color.
            // It will likely be too dark, but that's still probably better than not setting the color at all.
            // Convert from gamma to linear to try and account for the different color spaces between phong and PBR rendering.
            const auto materialColor = material->Element()->HasElement("diffuse") ? material->Diffuse() : material->Ambient();
            const AZ::Color baseColor = URDF::TypeConversions::ConvertColor(materialColor).GammaToLinear();

            for (auto& [id, materialAssignment] : overrides)
            {
                materialAssignment.m_propertyOverrides.emplace(AZ::Name("baseColor.color"), AZStd::any(baseColor));
            }
        }
    }

    static void OverrideMaterialTransparency(const sdf::Visual* visual, AZ::Render::MaterialAssignmentMap& overrides)
    {
        // Opacity: Use visual->transparency to set the material's opacity.
        if (visual->Element()->HasElement("transparency"))
        {
            const auto transparency = visual->Transparency();
            if (transparency > 0.0f)
            {
                // Override the material properties for every material used by the model.
                for (auto& [id, materialAssignment] : overrides)
                {
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("opacity.mode"), AZStd::any(AZStd::string("Blended")));
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("opacity.alphaSource"), AZStd::any(AZStd::string("None")));
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("opacity.factor"), AZStd::any(1.0f - transparency));
                }
            }
        }
    }

    static void OverrideMaterialEmissiveSettings(const sdf::Material* material, AZ::Render::MaterialAssignmentMap& overrides)
    {
        // Emissive: If an emissive color has been specified, enable emissive on the material and set the emissive color to the provided one.
        if (material->Element()->HasElement("emissive"))
        {
            // Get the color and convert from gamma to linear to try and account for the different color spaces between phong and PBR rendering.
            const auto materialColor = material->Emissive();
            const AZ::Color emissiveColor = URDF::TypeConversions::ConvertColor(materialColor).GammaToLinear();

            // It seems to be fairly common to have an emissive entry of black, which isn't emissive at all. 
            // Only enable the emissive color if it's a non-black value.
            if ((emissiveColor.GetR() > 0.0f) || (emissiveColor.GetG() > 0.0f) || (emissiveColor.GetB() > 0.0f))
            {
                for (auto& [id, materialAssignment] : overrides)
                {
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("emissive.enable"), AZStd::any(true));
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("emissive.color"), AZStd::any(emissiveColor));

                    // The URDF/SDF file doesn't specify an emissive intensity, just a color.
                    // We're arbitrarily using a value slightly higher than the default emissive intensity.
                    // This value was picked based on observations of emissive color behaviors in Gazebo.
                    // This intensity mostly preserves the color (though it lightens it a little) and
                    // potentially adds a little bit of lighting to the scene if Bloom or Diffuse Probe Grid also exist in the world.
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("emissive.intensity"), AZStd::any(5.5f));
                }
            }
        }
    }

    static void OverrideMaterialRoughness(const sdf::Material* material, AZ::Render::MaterialAssignmentMap& overrides)
    {
        // Metallic/Roughness: Try to use the shininess value for roughness if we have one, otherwise fall back to using the specular brightness.
        if (material->Element()->HasElement("shininess") || material->Element()->HasElement("specular"))
        {
            float shininess = 0.0f;
            float roughness = 0.0f;

            if (material->Element()->HasElement("shininess"))
            {
                // If we have a shininess value, we'll use it to set both metallic and roughness.
                // The shinier it is, the more metallic and less rough we'll make the result.
                shininess = material->Shininess();
                roughness = 1.0f - shininess;
            }
            else
            {
                // We don't have shininess, so use the specular color to estimate metallic/roughness values.

                // Convert the specular color into a brightness value. To get the brightness, we'll use the average of the three
                // color values. This isn't the most perceptually accurate choice, but specular brightness isn't the same as roughness
                // anyways, so it's hard to say what perceptual brightness model would cause any better or worse results here.
                // Another possibility to consider would be taking the max of the RGB values.
                const auto materialColor = material->Specular();
                const AZ::Color specularColor = URDF::TypeConversions::ConvertColor(materialColor);
                const float specularBrightness = (specularColor.GetR() + specularColor.GetG() + specularColor.GetB()) / 3.0f;
                roughness = 1.0f - specularBrightness;

                // Since specular color doesn't really speak to shininess, we'll arbitrarily scale down the specular brightness to
                // 1/4 of the total brightness to modulate the metallic reflectiveness a little, but not too much. Without this scaling,
                // a white specular color would always become fully metallic, perfectly smooth, and therefore fully reflective.
                // With the scaling, a white specular color will be perfectly smooth but only 25% metallic, so it will have some
                // reflectivity but not a lot.
                shininess = specularBrightness * 0.25f;
            }

            for (auto& [id, materialAssignment] : overrides)
            {
                materialAssignment.m_propertyOverrides.emplace(AZ::Name("metallic.factor"), AZStd::any(shininess));
                materialAssignment.m_propertyOverrides.emplace(AZ::Name("roughness.factor"), AZStd::any(roughness));
            }
        }
    }

    static void OverrideMaterialDoubleSided(const sdf::Material* material, AZ::Render::MaterialAssignmentMap& overrides)
    {
        // DoubleSided: The double_sided element converts directly over to the O3DE doubleSided material attribute.
        if (material->Element()->HasElement("double_sided"))
        {
            // The default material property value is one-sided, so only override the value if it should be double-sided.
            if (material->DoubleSided())
            {
                for (auto& [id, materialAssignment] : overrides)
                {
                    materialAssignment.m_propertyOverrides.emplace(AZ::Name("general.doubleSided"), AZStd::any(true));
                }
            }
        }
    }

    void VisualsMaker::AddMaterialForVisual(const sdf::Visual* visual, AZ::EntityId entityId, const AZ::Data::AssetId& assetId) const
    {
        auto material = visual->Material();

        if (!material)
        {
            // Material is optional, it might not appear on all visuals.
            return;
        }

        AZ_Assert(material->Element(), "Material has data but no Element pointer. Something unexpected has happened with the SDF parsing.");

        // Conversions from <material> in the file to O3DE are extremely imprecise because the data is going from a Phong model to PBR,
        // and there are no direct translations from one type of lighting model to the other. All of the conversions will create some
        // rough approximations of the source lighting data, but should hopefully provide a reasonable starting point for tuning the look.

        // Also, URDF/SDF files don't have a concept of overriding specific materials, so every material override generated
        // below will get applied to *all* materials for a mesh file.

        // First, force the model asset to get loaded into memory before adding the material component.
        // This is required so that we can get the default material map that will be used to override properties for each material.
        auto modelAsset = AZ::Data::AssetManager::Instance().GetAsset<AZ::RPI::ModelAsset>(assetId, AZ::Data::AssetLoadBehavior::Default);
        modelAsset.BlockUntilLoadComplete();

        AZ_Error("AddMaterial", modelAsset.IsReady(), "Trying to create materials for a model that couldn't load. The generated material overrides may not work correctly.");

        // Initialize the material component configuration to contain all of the material mappings from the model.
        AZ::Render::MaterialComponentConfig config;
        config.m_materials = AZ::Render::GetDefaultMaterialMapFromModelAsset(modelAsset);

        // Try to override all of the various material settings based on what's contained in the <material> and <visual> elements in the source file.
        OverrideScriptMaterial(material, config.m_materials);
        OverrideMaterialPbrSettings(material, m_urdfAssetsMapping, config.m_materials);
        OverrideMaterialBaseColor(material, config.m_materials);
        OverrideMaterialTransparency(visual, config.m_materials);
        OverrideMaterialEmissiveSettings(material, config.m_materials);
        OverrideMaterialRoughness(material, config.m_materials);
        OverrideMaterialDoubleSided(material, config.m_materials);

        // All the material overrides are in place, so get the entity, add the material component, and set its configuration to use the material overrides.
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "Entity ID for visual %s couldn't be found.", visual->Name().c_str());
        auto component = entity->CreateComponent(AZ::Render::EditorMaterialComponentTypeId);
        component->SetConfiguration(config);
    }
} // namespace ROS2
