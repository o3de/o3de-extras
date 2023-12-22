/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SourceAssetsStorage.h"
#include "AzCore/Outcome/Outcome.h"
#include "RobotImporterUtils.h"
#include <Atom/RPI.Reflect/Image/StreamingImageAsset.h>
#include <Atom/RPI.Reflect/Model/ModelAsset.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Serialization/Json/JsonImporter.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Asset/AssetSystemBus.h>
#include <AzToolsFramework/Asset/AssetUtils.h>
#include <PhysX/MeshAsset.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/DataTypes/GraphData/IMaterialData.h>
#include <SceneAPI/SceneCore/DataTypes/Groups/ISceneNodeGroup.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SceneAPI/SceneCore/Import/SceneImportSettings.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <SceneAPI/SceneData/Groups/ImportGroup.h>
#include <SceneAPI/SceneData/Groups/MeshGroup.h>
#include <SceneAPI/SceneData/Rules/UVsRule.h>
#include <Source/Pipeline/MeshGroup.h> // PhysX/Code/Source/Pipeline/MeshGroup.h

namespace ROS2::Utils
{

    //! A helper class that do no extend PhysX::Pipeline::MeshGroup functionality, but gives necessary setters.
    class UrdfPhysxMeshGroupHelper : public PhysX::Pipeline::MeshGroup
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

    //! Returns supported filenames by Asset Processor
    AZStd::vector<AZStd::string> GetSupportedExtensions()
    {
        struct Visitor : AZ::SettingsRegistryInterface::Visitor
        {
            using AZ::SettingsRegistryInterface::Visitor::Visit;
            void Visit(const AZ::SettingsRegistryInterface::VisitArgs&, AZStd::string_view value) override
            {
                m_supportedFileExtensions.emplace_back(value);
            }

            AZStd::vector<AZStd::string> m_supportedFileExtensions;
        };
        auto settingsRegistry = AZ::SettingsRegistry::Get();

        if (settingsRegistry == nullptr)
        {
            AZ_Warning("GetInterestingSourceAssetsCRC", false, "Global Settings Registry is not set.");
            return AZStd::vector<AZStd::string>();
        }

        using namespace AzToolsFramework::AssetUtils;
        Visitor assetImporterVisitor;
        settingsRegistry->Visit(
            assetImporterVisitor,
            AZ::SettingsRegistryInterface::FixedValueString("/O3DE/SceneAPI/AssetImporter/SupportedFileTypeExtensions"));
        return assetImporterVisitor.m_supportedFileExtensions;
    }

    /// Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZ::IO::Path& filename)
    {
        auto fileSize = AZ::IO::SystemFile::Length(filename.c_str());
        fileSize = AZStd::min(fileSize, 1024ull); // limit crc computation to first kilobyte
        if (fileSize == 0)
        {
            return AZ::Crc32();
        }
        AZStd::vector<char> buffer;
        buffer.resize_no_construct(fileSize + 1);
        buffer[fileSize] = '\0';
        if (!AZ::IO::SystemFile::Read(filename.c_str(), buffer.data(), fileSize))
        {
            return AZ::Crc32();
        }
        AZ::Crc32 r;
        r.Add(buffer.data(), fileSize);
        return r;
    }

    AZStd::vector<AZStd::string> GetProductAssets(const AZ::Uuid& sourceAssetUUID)
    {
        AZStd::vector<AZStd::string> productPaths;
        const auto importantTypes = AZStd::to_array<const AZ::TypeId>(
        {
            azrtti_typeid<AZ::RPI::StreamingImageAsset>(),
            azrtti_typeid<AZ::RPI::ModelAsset>(),
            azrtti_typeid<PhysX::Pipeline::MeshAsset>()
        });

        AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        bool ok{ false };
        AssetSysReqBus::BroadcastResult(ok, &AssetSysReqBus::Events::GetAssetsProducedBySourceUUID, sourceAssetUUID, productsAssetInfo);
        if (ok)
        {
            for (auto& product : productsAssetInfo)
            {
                for (auto& typeId : importantTypes)
                {
                    if (product.m_assetType == typeId)
                    {
                        AZStd::string assetPath;
                        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                            assetPath, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetPathById, product.m_assetId);
                        if (!assetPath.empty())
                        {
                            productPaths.emplace_back(AZStd::move(assetPath));
                            break;
                        }
                    }
                }
            }
        }

        return productPaths;
    }

    AZ::Data::AssetId GetProductAssetId(const AZ::Uuid& sourceAssetUUID, const AZ::TypeId typeId)
    {
        AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        bool ok{ false };
        AssetSysReqBus::BroadcastResult(ok, &AssetSysReqBus::Events::GetAssetsProducedBySourceUUID, sourceAssetUUID, productsAssetInfo);
        if (ok)
        {
            for (const auto& product : productsAssetInfo)
            {
                if (product.m_assetType == typeId)
                {
                    return product.m_assetId;
                }
            }
        }
        return {};
    }

    AZ::Data::AssetId GetImageProductAssetId(const AZ::Uuid& sourceAssetUUID)
    {
        return GetProductAssetId(sourceAssetUUID, azrtti_typeid<AZ::RPI::StreamingImageAsset>());
    }

    AZ::Data::AssetId GetModelProductAssetId(const AZ::Uuid& sourceAssetUUID)
    {
        return GetProductAssetId(sourceAssetUUID, azrtti_typeid<AZ::RPI::ModelAsset>());
    }

    AZ::Data::AssetId GetPhysXMeshProductAssetId(const AZ::Uuid& sourceAssetUUID)
    {
        return GetProductAssetId(sourceAssetUUID, azrtti_typeid<PhysX::Pipeline::MeshAsset>());
    }

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

        AZ::Crc32 crc = Utils::GetFileCRC(foundAsset.m_sourceAssetGlobalPath);
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

    AZStd::unordered_map<AZ::Crc32, AvailableAsset> GetInterestingSourceAssetsCRC()
    {
        // connect to database API
        const AZStd::vector<AZStd::string> InterestingExtensions = GetSupportedExtensions();
        AZStd::unordered_map<AZ::Crc32, AvailableAsset> availableAssets;

        AzToolsFramework::AssetDatabase::AssetDatabaseConnection assetDatabaseConnection;
        if (!assetDatabaseConnection.OpenDatabase())
        {
            AZ_Warning("GetInterestingSourceAssetsCRC", false, "Cannot open database");
            return {};
        }
        auto callback = [&availableAssets](AzToolsFramework::AssetDatabase::SourceDatabaseEntry& entry)
        {
            using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
            AvailableAsset foundAsset;
            foundAsset.m_sourceGuid = entry.m_sourceGuid;

            using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;

            // get source asset info
            bool sourceAssetFound{ false };
            AZ::Data::AssetInfo assetInfo;
            AZStd::string watchFolder;
            AssetSysReqBus::BroadcastResult(
                sourceAssetFound, &AssetSysReqBus::Events::GetSourceInfoBySourceUUID, entry.m_sourceGuid, assetInfo, watchFolder);
            if (!sourceAssetFound)
            {
                AZ_Warning("GetInterestingSourceAssetsCRC", false, "Cannot find source asset info for %s", entry.ToString().c_str());
                return true;
            }

            const auto fullSourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(assetInfo.m_relativePath);

            foundAsset.m_sourceAssetRelativePath = assetInfo.m_relativePath;
            foundAsset.m_sourceAssetGlobalPath = fullSourcePath.String();

            AZ::Crc32 crc = Utils::GetFileCRC(foundAsset.m_sourceAssetGlobalPath);
            if (crc == AZ::Crc32(0))
            {
                AZ_Warning(
                    "GetInterestingSourceAssetsCRC", false, "Zero CRC for source asset %s", foundAsset.m_sourceAssetGlobalPath.c_str());
                return true;
            }
            AZ_Printf("GetInterestingSourceAssetsCRC", "Found asset:");
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tm_sourceAssetRelativePath  : %s", foundAsset.m_sourceAssetRelativePath.c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tm_sourceAssetGlobalPath    : %s", foundAsset.m_sourceAssetGlobalPath.c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tm_productAssetRelativePath : %s", foundAsset.m_productAssetRelativePath.c_str());
            AZ_Printf(
                "GetInterestingSourceAssetsCRC",
                "\tm_sourceGuid               : %s",
                foundAsset.m_sourceGuid.ToString<AZStd::string>().c_str());
            AZ_Printf("GetInterestingSourceAssetsCRC", "\tcrc                        : %d", crc);

            auto availableAssetIt = availableAssets.find(crc);
            if (availableAssetIt != availableAssets.end())
            {
                AZ_Warning(
                    "GetInterestingSourceAssetsCRC", false, "Asset already in database : %s ", foundAsset.m_sourceAssetGlobalPath.c_str());
                AZ_Warning(
                    "GetInterestingSourceAssetsCRC", false, "Found asset : %s ", availableAssetIt->second.m_sourceAssetGlobalPath.c_str());
            }
            else
            {
                availableAssets.insert({ crc, foundAsset });
            }
            return true;
        };

        for (auto& extension : InterestingExtensions)
        {
            assetDatabaseConnection.QuerySourceLikeSourceName(
                extension.c_str(), AzToolsFramework::AssetDatabase::AssetDatabaseConnection::LikeType::EndsWith, callback);
        }
        return availableAssets;
    }

    UrdfAssetMap CopyReferencedAssetsAndCreateAssetMap(
        const AssetFilenameReferences& assetFilenames,
        const AZStd::string& urdfFilename,
        const SdfAssetBuilderSettings& sdfBuilderSettings,
        AZStd::string_view outputDirSuffix,
        AZ::IO::FileIOBase* fileIO)
    {
        auto urdfAssetMap = CreateAssetMap(assetFilenames, urdfFilename, sdfBuilderSettings);
        AZStd::mutex urdfAssetMapMutex;
        if (urdfAssetMap.empty())
        {
            return urdfAssetMap;
        }

        auto destDirectory = PrepareImportedAssetsDest(urdfFilename, outputDirSuffix, fileIO);
        if (!destDirectory.IsSuccess())
        {
            return urdfAssetMap;
        }

        AZStd::unordered_map<AZ::IO::Path, unsigned int> duplicatedFilenames;
        for (auto& [unresolvedFileName, urdfAsset] : urdfAssetMap)
        {
            if (duplicatedFilenames.contains(unresolvedFileName))
            {
                duplicatedFilenames[unresolvedFileName]++;
            }
            else
            {
                duplicatedFilenames[unresolvedFileName] = 0;
            }
            CopyReferencedAsset(unresolvedFileName, destDirectory.GetValue(), urdfAsset, duplicatedFilenames[unresolvedFileName]);
        }
        Utils::RemoveTmpDir(destDirectory.GetValue().importDirectoryTmp);

        return urdfAssetMap;
    }

    UrdfAssetMap FindReferencedAssets(
        const AssetFilenameReferences& assetFilenames,
        const AZStd::string& urdfFilename,
        const SdfAssetBuilderSettings& sdfBuilderSettings)
    {
        auto amentPrefixPath = Utils::GetAmentPrefixPath();

        UrdfAssetMap urdfToAsset;
        for (const auto& [assetPath, assetReferenceType] : assetFilenames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = assetPath;
            asset.m_resolvedUrdfPath =
                Utils::ResolveAssetPath(asset.m_urdfPath, AZ::IO::PathView(urdfFilename), amentPrefixPath, sdfBuilderSettings);
            asset.m_urdfFileCRC = Utils::GetFileCRC(asset.m_resolvedUrdfPath);
            urdfToAsset.emplace(assetPath, AZStd::move(asset));
        }

        if (!urdfToAsset.empty())
        {
            AZStd::unordered_map<AZ::Crc32, AvailableAsset> availableAssets = Utils::GetInterestingSourceAssetsCRC();

            // Search for suitable mappings by comparing checksum
            for (auto it = urdfToAsset.begin(); it != urdfToAsset.end(); it++)
            {
                Utils::UrdfAsset& asset = it->second;
                auto found_source_asset = availableAssets.find(asset.m_urdfFileCRC);
                if (found_source_asset != availableAssets.end())
                {
                    asset.m_availableAssetInfo = found_source_asset->second;
                }
            }
        }

        return urdfToAsset;
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
            AZStd::shared_ptr<UrdfPhysxMeshGroupHelper> physxDataMeshGroup = AZStd::make_shared<UrdfPhysxMeshGroupHelper>();
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

    UrdfAssetMap CreateAssetMap(
        const AssetFilenameReferences& assetFilenames, const AZStd::string& urdfFilename, const SdfAssetBuilderSettings& sdfBuilderSettings)
    {
        UrdfAssetMap urdfAssetMap;
        if (assetFilenames.empty())
        {
            return urdfAssetMap;
        }

        auto amentPrefixPath = Utils::GetAmentPrefixPath();
        for (const auto& [unresolvedFileName, assetReferenceType] : assetFilenames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = unresolvedFileName;
            asset.m_resolvedUrdfPath =
                Utils::ResolveAssetPath(unresolvedFileName, AZ::IO::PathView(urdfFilename), amentPrefixPath, sdfBuilderSettings);
            asset.m_urdfFileCRC = AZ::Crc32();
            asset.m_assetReferenceType = assetReferenceType;
            asset.m_unresolvedFileName = unresolvedFileName;
            urdfAssetMap.emplace(unresolvedFileName, AZStd::move(asset));
        }

        return urdfAssetMap;
    }

    AZ::Outcome<ImportedAssetsDest> PrepareImportedAssetsDest(
        const AZStd::string& urdfFilename, AZStd::string_view outputDirSuffix, AZ::IO::FileIOBase* fileIO)
    {
        AZ_Assert(fileIO, "No FileIO instance");
        AZ::Crc32 urdfFileCrc;
        urdfFileCrc.Add(urdfFilename);
        const AZ::IO::Path urdfPath(urdfFilename);

        // By naming the temp directory '$tmp_*', the default configuration in AssetProcessorPlatformConfig.setreg will
        // exclude these files from processing.
        const AZStd::string directoryNameTmp = AZStd::string::format("$tmp_%u.tmp", AZ::u32(urdfFileCrc));
        const auto directoryNameDst = AZ::IO::FixedMaxPathString::format(
            "%u_%.*s%.*s", AZ::u32(urdfFileCrc), AZ_PATH_ARG(urdfPath.Stem()), AZ_STRING_ARG(outputDirSuffix));

        const AZ::IO::Path importDirectoryTmp = AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "UrdfImporter" / directoryNameTmp;
        const AZ::IO::Path importDirectoryDst = AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "UrdfImporter" / directoryNameDst;

        fileIO->DestroyPath(importDirectoryTmp.c_str());
        const auto outcomeCreateDstDir = fileIO->CreatePath(importDirectoryDst.c_str());
        const auto outcomeCreateTmpDir = fileIO->CreatePath(importDirectoryTmp.c_str());
        AZ_Error("CopyAssetForURDF", outcomeCreateDstDir, "Cannot create destination directory : %s", importDirectoryDst.c_str());
        AZ_Error("CopyAssetForURDF", outcomeCreateTmpDir, "Cannot create temporary directory : %s", importDirectoryTmp.c_str());

        if (!outcomeCreateDstDir || !outcomeCreateTmpDir)
        {
            if (outcomeCreateDstDir)
            {
                fileIO->DestroyPath(importDirectoryDst.c_str());
            }
            if (outcomeCreateTmpDir)
            {
                fileIO->DestroyPath(importDirectoryTmp.c_str());
            }
            return AZ::Failure();
        }

        const ImportedAssetsDest importedAssetsDest = { importDirectoryTmp, importDirectoryDst };

        return AZ::Success(importedAssetsDest);
    }

    AZ::Outcome<bool> RemoveTmpDir(const AZ::IO::Path $tmpDir, AZ::IO::FileIOBase* fileIO)
    {
        AZ_Assert(fileIO, "No FileIO instance");
        const auto outcomeRemoveTmpDir = fileIO->DestroyPath($tmpDir.c_str());
        if (!outcomeRemoveTmpDir)
        {
            AZ_Error("CopyAssetForURDF", false, "Cannot remove temporary directory : %s", $tmpDir.c_str());
            return AZ::Failure();
        }
        return AZ::Success(true);
    }

    CopyStatus CopyReferencedAsset(
        const AZ::IO::Path& unresolvedFileName,
        const ImportedAssetsDest& importedAssetsDest,
        Utils::UrdfAsset& urdfAsset,
        unsigned int duplicationCounter,
        AZ::IO::FileIOBase* fileIO)
    {
        if (urdfAsset.m_resolvedUrdfPath.empty())
        {
            AZ_Warning("CopyAssetForURDF", false, "There is no resolved path for %s", unresolvedFileName.c_str());
            return CopyStatus::Unresolvable;
        }

        AZStd::string filename = urdfAsset.m_resolvedUrdfPath.Filename().String();
        if (duplicationCounter > 0)
        {
            AZStd::string stem = urdfAsset.m_resolvedUrdfPath.Stem().String();
            AZStd::string extension = urdfAsset.m_resolvedUrdfPath.Extension().String();
            filename = AZStd::string::format("%s_dup_%u%s", stem.c_str(), duplicationCounter, extension.c_str());
        }

        AZ::IO::Path targetPathAssetDst(importedAssetsDest.importDirectoryDst / filename);
        AZ::IO::Path targetPathAssetTmp(importedAssetsDest.importDirectoryTmp / filename);

        AZ::IO::Path targetPathAssetInfo(targetPathAssetDst.Native() + ".assetinfo");

        bool targetAssetExists = fileIO->Exists(targetPathAssetDst.c_str());
        if (!targetAssetExists)
        {
            urdfAsset.m_copyStatus = CopyStatus::Copying;

            // copy mesh file to temporary location ignored by AP
            const auto outcomeCopyTmp = fileIO->Copy(urdfAsset.m_resolvedUrdfPath.c_str(), targetPathAssetTmp.c_str());
            AZ_Printf(
                "CopyAssetForURDF",
                "Copy %s to %s, result: %d",
                urdfAsset.m_resolvedUrdfPath.c_str(),
                targetPathAssetTmp.c_str(),
                outcomeCopyTmp.GetResultCode());

            if (outcomeCopyTmp)
            {
                // call FlushIOOfAsset to ensure the asset processor is aware of the new file
                FlushIOOfAsset(targetPathAssetTmp);

                const bool needsVisual =
                    (urdfAsset.m_assetReferenceType & ReferencedAssetType::VisualMesh) == ReferencedAssetType::VisualMesh;
                const bool needsCollider =
                    (urdfAsset.m_assetReferenceType & ReferencedAssetType::ColliderMesh) == ReferencedAssetType::ColliderMesh;
                const bool isMeshFile = (needsVisual || needsCollider);

                // if the asset is a mesh, create asset info at destination location using the temporary mesh file
                const bool assetInfoOk =
                    isMeshFile ? CreateSceneManifest(targetPathAssetTmp, targetPathAssetInfo, needsCollider, needsVisual) : true;

                if (assetInfoOk)
                {
                    // copy additional assets such as textures directly to destination location
                    if (isMeshFile)
                    {
                        const auto& meshTextureAssets = Utils::GetMeshTextureAssets(targetPathAssetTmp);
                        for (const auto& unresolvedAssetPath : meshTextureAssets)
                        {
                            // Manifest returns local path in Project's directory temp folder
                            const AZ::IO::Path assetLocalPath(AZ::IO::Path(AZ::IO::Path(AZ::Utils::GetProjectPath()) / unresolvedAssetPath)
                                                                  .LexicallyRelative(importedAssetsDest.importDirectoryTmp));

                            const AZ::IO::Path assetFullPathSrc(AZ::IO::Path(urdfAsset.m_resolvedUrdfPath.ParentPath()) / assetLocalPath);
                            const AZ::IO::Path assetFullPathDst(importedAssetsDest.importDirectoryDst / assetLocalPath);

                            const auto outcomeMkdir = fileIO->CreatePath(AZ::IO::Path(assetFullPathDst.ParentPath()).c_str());
                            if (!outcomeMkdir)
                            {
                                break;
                            }

                            const auto outcomeCopy = fileIO->Copy(assetFullPathSrc.c_str(), assetFullPathDst.c_str());
                            if (!outcomeCopy)
                            {
                                urdfAsset.m_copyStatus = CopyStatus::Failed;
                            }
                        }
                    }

                    // move asset file from temporary location to destination location
                    const auto outcomeMoveDst = fileIO->Rename(targetPathAssetTmp.c_str(), targetPathAssetDst.c_str());
                    AZ_Printf(
                        "CopyAssetForURDF",
                        "Rename file %s to %s, result: %d",
                        targetPathAssetTmp.c_str(),
                        targetPathAssetDst.c_str(),
                        outcomeMoveDst.GetResultCode());

                    // call FlushIOOfAsset to ensure the asset processor is aware of the new file
                    FlushIOOfAsset(targetPathAssetDst);

                    urdfAsset.m_copyStatus = outcomeMoveDst ? CopyStatus::Copied : CopyStatus::Failed;
                }
            }
            else
            {
                urdfAsset.m_copyStatus = CopyStatus::Failed;
            }
        }
        else
        {
            AZ_Printf("CopyAssetForURDF", "File %s already exists, omitting import", targetPathAssetDst.c_str());
            urdfAsset.m_copyStatus = CopyStatus::Exists;
        }

        if (urdfAsset.m_copyStatus == CopyStatus::Exists || urdfAsset.m_copyStatus == CopyStatus::Copied)
        {
            urdfAsset.m_availableAssetInfo = Utils::GetAvailableAssetInfo(targetPathAssetDst.String());
        }
        urdfAsset.m_urdfPath = "";
        urdfAsset.m_urdfFileCRC = AZ::Crc32();

        return urdfAsset.m_copyStatus;
    }

    AZStd::unordered_set<AZ::IO::Path> GetMeshTextureAssets(const AZ::IO::Path& sourceMeshAssetPath)
    {
        AZStd::shared_ptr<AZ::SceneAPI::Containers::Scene> scene;
        AZ::SceneAPI::Events::SceneSerializationBus::BroadcastResult(
            scene, &AZ::SceneAPI::Events::SceneSerialization::LoadScene, sourceMeshAssetPath.c_str(), AZ::Uuid::CreateNull(), "");
        if (!scene)
        {
            AZ_Error("GetMeshTextureAssets", false, "Error loading mesh assets. Invalid scene: %s", sourceMeshAssetPath.c_str());
            return AZStd::unordered_set<AZ::IO::Path>();
        }

        AZStd::unordered_set<AZ::IO::Path> assetsFilepaths;

        // Look for material files
        static const AZStd::array<AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType, 9> allTextureTypes{
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::Diffuse,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::Specular,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::Bump,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::Normal,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::Metallic,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::Roughness,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::AmbientOcclusion,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::Emissive,
            AZ::SceneAPI::DataTypes::IMaterialData::TextureMapType::BaseColor
        };
        auto view =
            AZ::SceneAPI::Containers::MakeDerivedFilterView<AZ::SceneAPI::DataTypes::IMaterialData>(scene->GetGraph().GetContentStorage());
        for (const auto& material : view)
        {
            for (auto textureType : allTextureTypes)
            {
                if (const AZ::IO::Path filePath(material.GetTexture(textureType)); !filePath.empty())
                {
                    assetsFilepaths.emplace(filePath);
                }
            }
        }
        return assetsFilepaths;
    }

    AzFramework::AssetSystem::AssetStatus FlushIOOfAsset(const AZ::IO::Path& path)
    {
        AzFramework::AssetSystem::AssetStatus assetStatus = AzFramework::AssetSystem::AssetStatus::AssetStatus_Unknown;
        AzFramework::AssetSystemRequestBus::BroadcastResult(
            assetStatus, &AzFramework::AssetSystem::AssetSystemRequests::GetAssetStatus_FlushIO, path.c_str());
        AZ_Warning(
            "CopyAssetForURDF",
            assetStatus != AzFramework::AssetSystem::AssetStatus::AssetStatus_Unknown,
            "Asset processor did not recognize the new file %s.",
            path.c_str());

        return assetStatus;
    }
} // namespace ROS2::Utils
