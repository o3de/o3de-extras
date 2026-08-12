/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SceneManifestBuilder.h"
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <RobotImporter/Assets/AssetLookup.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <SceneAPI/SceneData/Groups/ImportGroup.h>
#include <SceneAPI/SceneData/Groups/MeshGroup.h>
#include <SceneAPI/SceneData/Rules/UVsRule.h>
#include <Source/Pipeline/MeshGroup.h> // PhysX/Code/Source/Pipeline/MeshGroup.h

namespace ROS2RobotImporter::Assets
{

    //! A helper class that do no extend PhysX::Pipeline::MeshGroup functionality, but gives necessary setters.
    class PhysxMeshGroupHelper : public PhysX::Pipeline::MeshGroup
    {
    public:
        void SetIsDecomposeMeshes(bool decompose)
        {
            m_decomposeMeshes = decompose;
        }
        void SetMeshExportMethod(PhysX::Pipeline::MeshExportMethod method)
        {
            m_exportMethod = method;
        }
    };

    AvailableAsset GetAvailableAssetInfo(const AZStd::string& globalSourceAssetPath)
    {
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        AvailableAsset foundAsset;

        // get source asset info
        bool sourceAssetFound{ false };
        AZ::Data::AssetInfo assetInfo;
        AZStd::string watchFolder;
        AssetSysReqBus::BroadcastResult(
            sourceAssetFound, &AssetSysReqBus::Events::GetSourceInfoBySourcePath, globalSourceAssetPath.c_str(), assetInfo, watchFolder);
        if (!sourceAssetFound)
        {
            AZ_Trace("GetAvailableAssetInfo", "Source asset info not found for: %s", globalSourceAssetPath.c_str());
            return foundAsset;
        }

        foundAsset.m_sourceGuid = assetInfo.m_assetId.m_guid;

        foundAsset.m_sourceAssetRelativePath = assetInfo.m_relativePath;
        foundAsset.m_sourceAssetGlobalPath = globalSourceAssetPath;

        AZ::Crc32 crc = Assets::GetFileCRC(foundAsset.m_sourceAssetGlobalPath);
        if (crc == AZ::Crc32(0))
        {
            AZ_Warning("GetInterestingSourceAssetsCRC", false, "Zero CRC for source asset %s", foundAsset.m_sourceAssetGlobalPath.c_str());
            return foundAsset;
        }

        AZ_Printf("GetAvailableAssetInfo", "Found asset:");
        AZ_Printf("GetAvailableAssetInfo", "\tm_sourceAssetRelativePath  : %s", foundAsset.m_sourceAssetRelativePath.c_str());
        AZ_Printf("GetAvailableAssetInfo", "\tm_sourceAssetGlobalPath    : %s", foundAsset.m_sourceAssetGlobalPath.c_str());
        AZ_Printf("GetAvailableAssetInfo", "\tm_productAssetRelativePath : %s", foundAsset.m_productAssetRelativePath.c_str());
        AZ_Printf("GetAvailableAssetInfo", "\tm_sourceGuid               : %s", foundAsset.m_sourceGuid.ToString<AZStd::string>().c_str());

        return foundAsset;
    }

    bool CreateSceneManifest(const AZ::IO::Path& sourceAssetPath, const AZ::IO::Path& assetInfoFile, const bool collider, const bool visual)
    {
        // Start with a default set of import settings.
        AZ::SceneAPI::SceneImportSettings importSettings;

        // OBJ files used for robotics might be authored with hundreds or thousands of tiny groups of meshes. By default,
        // AssImp splits each object or group in an OBJ into a separate submesh, creating an extremely non-optimal result.
        // By turning on the AssImp options to optimize the scene and the meshes, all of these submeshes will get recombined
        // back into just a few submeshes.
        if (sourceAssetPath.Extension() == ".obj")
        {
            importSettings.m_optimizeScene = true;
            importSettings.m_optimizeMeshes = true;
        }

        // Set the import settings into the settings registry.
        // This needs to happen before calling LoadScene so that the AssImp import settings are applied to the scene being
        // read into memory. These settings affect the list of scene nodes referenced by the MeshGroup and PhysXGroup settings,
        // so it's important to apply them here to get the proper node lists.
        if (AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get(); settingsRegistry)
        {
            settingsRegistry->SetObject(AZ::SceneAPI::DataTypes::IImportGroup::SceneImportSettingsRegistryKey, importSettings);
        }

        AZ_Printf("CreateSceneManifest", "Creating manifest for asset %s at : %s ", sourceAssetPath.c_str(), assetInfoFile.c_str());
        AZStd::shared_ptr<AZ::SceneAPI::Containers::Scene> scene;
        AZ::SceneAPI::Events::SceneSerializationBus::BroadcastResult(
            scene, &AZ::SceneAPI::Events::SceneSerialization::LoadScene, sourceAssetPath.c_str(), AZ::Uuid::CreateNull(), "");
        if (!scene)
        {
            AZ_Error("CreateSceneManifest", false, "Error loading collider. Invalid scene: %s", sourceAssetPath.c_str());
            return false;
        }

        AZ::SceneAPI::Containers::SceneManifest& manifest = scene->GetManifest();
        auto valueStorage = manifest.GetValueStorage();
        if (valueStorage.empty())
        {
            AZ_Error("CreateSceneManifest", false, "Error loading collider. Invalid value storage: %s", sourceAssetPath.c_str());
            return false;
        }

        // remove default configuration to avoid procedural prefab creation
        AZStd::vector<AZStd::shared_ptr<AZ::SceneAPI::DataTypes::IManifestObject>> toDelete;
        for (size_t i = 0; i < manifest.GetEntryCount(); i++)
        {
            AZStd::shared_ptr<AZ::SceneAPI::DataTypes::IManifestObject> obj = manifest.GetValue(i);
            toDelete.push_back(obj);
        }

        for (auto obj : toDelete)
        {
            AZ_Printf("CreateSceneManifest", "Deleting %s", obj->RTTI_GetType().ToString<AZStd::string>().c_str());
            manifest.RemoveEntry(obj);
        }

        // Create an entry for the import settings. This will contain default settings for most mesh files,
        // but will enable the import optimization settings for OBJ files.
        auto sceneDataImportGroup = AZStd::make_shared<AZ::SceneAPI::SceneData::ImportGroup>();
        sceneDataImportGroup->SetImportSettings(importSettings);
        manifest.AddEntry(sceneDataImportGroup);

        if (visual)
        {
            AZStd::shared_ptr<AZ::SceneAPI::SceneData::MeshGroup> sceneDataMeshGroup =
                AZStd::make_shared<AZ::SceneAPI::SceneData::MeshGroup>();

            // select all nodes to this mesh group
            AZ::SceneAPI::Utilities::SceneGraphSelector::SelectAll(scene->GetGraph(), sceneDataMeshGroup->GetSceneNodeSelectionList());

            // enable auto-generation of UVs
            sceneDataMeshGroup->GetRuleContainer().AddRule(AZStd::make_shared<AZ::SceneAPI::SceneData::UVsRule>());

            manifest.AddEntry(sceneDataMeshGroup);
        }

        if (collider)
        {
            AZStd::shared_ptr<PhysxMeshGroupHelper> physxDataMeshGroup = AZStd::make_shared<PhysxMeshGroupHelper>();
            physxDataMeshGroup->SetIsDecomposeMeshes(true);
            physxDataMeshGroup->SetMeshExportMethod(PhysX::Pipeline::MeshExportMethod::Convex);

            // select all nodes to this mesh group
            AZ::SceneAPI::Utilities::SceneGraphSelector::SelectAll(scene->GetGraph(), physxDataMeshGroup->GetSceneNodeSelectionList());

            manifest.AddEntry(physxDataMeshGroup);
        }

        // Update assetinfo
        AZ::SceneAPI::Events::ProcessingResultCombiner result;
        AZ::SceneAPI::Events::AssetImportRequestBus::BroadcastResult(
            result,
            &AZ::SceneAPI::Events::AssetImportRequest::UpdateManifest,
            *scene,
            AZ::SceneAPI::Events::AssetImportRequest::ManifestAction::Update,
            AZ::SceneAPI::Events::AssetImportRequest::RequestingApplication::Editor);

        if (result.GetResult() != AZ::SceneAPI::Events::ProcessingResult::Success)
        {
            AZ_Trace("CreateSceneManifest", "Scene updated\n");
            return false;
        }

        scene->GetManifest().SaveToFile(assetInfoFile.Native());
        AZ_Printf("CreateSceneManifest", "Saving scene manifest to %s\n", assetInfoFile.c_str());

        return true;
    }

    bool CreateSceneManifest(const AZ::IO::Path& sourceAssetPath, const bool collider, const bool visual)
    {
        return CreateSceneManifest(sourceAssetPath, sourceAssetPath.Native() + ".assetinfo", collider, visual);
    }

} // namespace ROS2RobotImporter::Assets
