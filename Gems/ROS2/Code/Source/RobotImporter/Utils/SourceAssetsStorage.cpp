/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SourceAssetsStorage.h"
#include "RobotImporterUtils.h"
#include <AzCore/IO/FileIO.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Asset/AssetSystemBus.h>
#include <AzToolsFramework/Asset/AssetUtils.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/DataTypes/Groups/ISceneNodeGroup.h>
#include <SceneAPI/SceneCore/Events/AssetImportRequest.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SceneAPI/SceneCore/Utilities/SceneGraphSelector.h>
#include <SceneAPI/SceneData/Groups/MeshGroup.h>
#include <SceneAPI/SceneData/Rules/UVsRule.h>
#include <Source/Pipeline/MeshGroup.h> // PhysX/Code/Source/Pipeline/MeshGroup.h

namespace ROS2::Utils
{

    //! A helper class that do no extend PhysX::Pipeline::MeshGroup functionality, but gives neccessary setters.
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

    AZStd::string GetProductAsset(const AZ::Uuid& sourceAssetUUID, const AZ::TypeId typeId)
    {
        AZStd::vector<AZ::Data::AssetInfo> productsAssetInfo;
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
        bool ok{ false };
        AssetSysReqBus::BroadcastResult(ok, &AssetSysReqBus::Events::GetAssetsProducedBySourceUUID, sourceAssetUUID, productsAssetInfo);
        if (ok)
        {
            for (auto& product : productsAssetInfo)
            {
                if (product.m_assetType == typeId)
                {
                    AZStd::string assetPath;
                    AZ::Data::AssetCatalogRequestBus::BroadcastResult(
                        assetPath, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetPathById, product.m_assetId);
                    return assetPath;
                }
            }
        }
        return "";
    }

    AZStd::string GetModelProductAsset(const AZ::Uuid& sourceAssetUUID)
    {
        return GetProductAsset(sourceAssetUUID, AZ::TypeId("{2C7477B6-69C5-45BE-8163-BCD6A275B6D8}")); // AZ::RPI::ModelAsset;
    }

    AZStd::string GetPhysXMeshProductAsset(const AZ::Uuid& sourceAssetUUID)
    {
        return GetProductAsset(sourceAssetUUID, AZ::TypeId("{7A2871B9-5EAB-4DE0-A901-B0D2C6920DDB}")); // PhysX::Pipeline::MeshAsset
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

    AZ::Data::AssetId GetModelProductAssetId(const AZ::Uuid& sourceAssetUUID)
    {
        return GetProductAssetId(sourceAssetUUID, AZ::TypeId("{2C7477B6-69C5-45BE-8163-BCD6A275B6D8}")); // AZ::RPI::ModelAsset;
    }

    AZ::Data::AssetId GetPhysXMeshProductAssetId(const AZ::Uuid& sourceAssetUUID)
    {
        return GetProductAssetId(sourceAssetUUID, AZ::TypeId("{7A2871B9-5EAB-4DE0-A901-B0D2C6920DDB}")); // PhysX::Pipeline::MeshAsset
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

    UrdfAssetMap CopyAssetForURDFAndCreateAssetMap(
        const AZStd::unordered_set<AZStd::string>& meshesFilenames,
        const AZStd::string& urdfFilename,
        const AZStd::unordered_set<AZStd::string>& colliders,
        const AZStd::unordered_set<AZStd::string>& visuals,
        AZStd::string_view outputDirSuffix,
        AZ::IO::FileIOBase* fileIO)
    {
        auto enviromentalVariable = std::getenv("AMENT_PREFIX_PATH");
        AZ_Error("UrdfAssetMap", enviromentalVariable, "AMENT_PREFIX_PATH is not found.");

        UrdfAssetMap urdfAssetMap;
        if (meshesFilenames.empty())
        {
            return urdfAssetMap;
        }

        //! Maps the unresolved urdf path to global path
        AZStd::unordered_map<AZStd::string, AZ::IO::Path> copiedFiles;

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
            return urdfAssetMap;
        }
        AZStd::string amentPrefixPath{ enviromentalVariable };
        AZStd::set<AZStd::string> files;

        for (const auto& unresolvedUrfFileName : meshesFilenames)
        {
            auto resolved =
                Utils::ResolveURDFPath(unresolvedUrfFileName, AZ::IO::PathView(urdfFilename), AZ::IO::PathView(amentPrefixPath));
            if (resolved.empty())
            {
                AZ_Warning("CopyAssetForURDF", false, "There is not resolved path for %s", unresolvedUrfFileName.c_str());
                continue;
            }

            AZ::IO::Path resolvedPath(resolved);
            const bool needsVisual = visuals.contains(unresolvedUrfFileName);
            const bool needsCollider = colliders.contains(unresolvedUrfFileName);

            AZ::IO::Path targetPathAssetDst(importDirectoryDst / resolvedPath.Filename());
            AZ::IO::Path targetPathAssetTmp(importDirectoryTmp / resolvedPath.Filename());

            AZ::IO::Path targetPathAssetInfo(targetPathAssetDst.Native() + ".assetinfo");

            if (!fileIO->Exists(targetPathAssetDst.c_str()))
            {
                // copy mesh file to temporary location ingored by AP
                const auto outcomeCopyTmp = fileIO->Copy(resolvedPath.c_str(), targetPathAssetTmp.c_str());
                AZ_Printf(
                    "CopyAssetForURDF",
                    "Copy %s to %s, result: %d",
                    resolvedPath.c_str(),
                    targetPathAssetTmp.c_str(),
                    outcomeCopyTmp.GetResultCode());
                if (outcomeCopyTmp)
                {
                    // create asset info at destination location using the temporary mesh file
                    const bool assetInfoOk =
                        CreateSceneManifest(targetPathAssetTmp.String(), targetPathAssetInfo.String(), needsCollider, needsVisual);

                    if (assetInfoOk)
                    {
                        // move mesh file from temporary location to destination location
                        const auto outcomeMoveDst = fileIO->Rename(targetPathAssetTmp.c_str(), targetPathAssetDst.c_str());
                        AZ_Printf(
                            "CopyAssetForURDF",
                            "Rename file %s to %s, result: %d",
                            targetPathAssetTmp.c_str(),
                            targetPathAssetDst.c_str(),
                            outcomeMoveDst.GetResultCode());

                        // call GetAssetStatus_FlushIO to ensure the asset processor is aware of the new file
                        AzFramework::AssetSystem::AssetStatus copiedAssetStatus =
                            AzFramework::AssetSystem::AssetStatus::AssetStatus_Unknown;
                        AzFramework::AssetSystemRequestBus::BroadcastResult(
                            copiedAssetStatus,
                            &AzFramework::AssetSystem::AssetSystemRequests::GetAssetStatus_FlushIO,
                            targetPathAssetDst.c_str());
                        AZ_Warning(
                            "CopyAssetForURDF",
                            copiedAssetStatus != AzFramework::AssetSystem::AssetStatus::AssetStatus_Unknown,
                            "Asset processor did not recognize the new file %s.",
                            targetPathAssetDst.c_str());

                        if (outcomeMoveDst)
                        {
                            copiedFiles[unresolvedUrfFileName] = targetPathAssetDst.String();
                        }
                    };
                }
            }
            else
            {
                AZ_Printf("CopyAssetForURDF", "File %s already exists, omitting import", targetPathAssetDst.c_str());
                copiedFiles[unresolvedUrfFileName] = targetPathAssetDst.String();
            }

            Utils::UrdfAsset asset;
            asset.m_urdfPath = urdfFilename;
            asset.m_resolvedUrdfPath =
                Utils::ResolveURDFPath(unresolvedUrfFileName, AZ::IO::PathView(urdfFilename), AZ::IO::PathView(amentPrefixPath));
            asset.m_urdfFileCRC = AZ::Crc32();
            urdfAssetMap.emplace(unresolvedUrfFileName, AZStd::move(asset));
        }

        fileIO->DestroyPath(importDirectoryTmp.c_str());
        for (const auto& copied : copiedFiles)
        {
            AZ_Printf("CopyAssetForURDF", " %s is copied to %s", copied.first.c_str(), copied.second.c_str());
        }

        // add available asset info
        for (const auto& [unresolvedUrfFileName, sourceAssetGlobalPath] : copiedFiles)
        {
            AZ_Assert(
                urdfAssetMap.contains(unresolvedUrfFileName), "urdfAssetMap should contain urdf path %s", unresolvedUrfFileName.c_str());
            urdfAssetMap[unresolvedUrfFileName].m_availableAssetInfo = Utils::GetAvailableAssetInfo(sourceAssetGlobalPath.String());
        }

        return urdfAssetMap;
    }

    UrdfAssetMap FindAssetsForUrdf(const AZStd::unordered_set<AZStd::string>& meshesFilenames, const AZStd::string& urdfFilename)
    {
        // Support reading the AMENT_PREFIX_PATH environment variable on Unix/Windows platforms
        auto StoreAmentPrefixPath = [](char* buffer, size_t size) -> size_t
        {
            auto getEnvOutcome = AZ::Utils::GetEnv(AZStd::span(buffer, size), "AMENT_PREFIX_PATH");
            return getEnvOutcome ? getEnvOutcome.GetValue().size() : 0;
        };
        AZStd::fixed_string<4096> amentPrefixPath;
        amentPrefixPath.resize_and_overwrite(amentPrefixPath.capacity(), StoreAmentPrefixPath);
        AZ_Error("UrdfAssetMap", !amentPrefixPath.empty(), "AMENT_PREFIX_PATH is not found.");

        UrdfAssetMap urdfToAsset;
        for (const auto& t : meshesFilenames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = t;
            asset.m_resolvedUrdfPath =
                Utils::ResolveURDFPath(asset.m_urdfPath, AZ::IO::PathView(urdfFilename), AZ::IO::PathView(amentPrefixPath));
            asset.m_urdfFileCRC = Utils::GetFileCRC(asset.m_resolvedUrdfPath);
            urdfToAsset.emplace(t, AZStd::move(asset));
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

    bool CreateSceneManifest(const AZ::IO::Path& sourceAssetPath, const AZ::IO::Path& assetInfoFile, bool collider, bool visual)
    {
        const auto& azMeshPath = sourceAssetPath;
        AZ_Printf("CreateSceneManifest", "Creating manifest for asset %s at : %s ", sourceAssetPath.c_str(), assetInfoFile.c_str());
        AZStd::shared_ptr<AZ::SceneAPI::Containers::Scene> scene;
        AZ::SceneAPI::Events::SceneSerializationBus::BroadcastResult(
            scene, &AZ::SceneAPI::Events::SceneSerialization::LoadScene, azMeshPath.c_str(), AZ::Uuid::CreateNull(), "");
        if (!scene)
        {
            AZ_Error("CreateSceneManifest", false, "Error loading collider. Invalid scene: %s", azMeshPath.c_str());
            return false;
        }

        AZ::SceneAPI::Containers::SceneManifest& manifest = scene->GetManifest();
        auto valueStorage = manifest.GetValueStorage();
        if (valueStorage.empty())
        {
            AZ_Error("CreateSceneManifest", false, "Error loading collider. Invalid value storage: %s", azMeshPath.c_str());
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

    bool CreateSceneManifest(const AZ::IO::Path& sourceAssetPath, bool collider, bool visual)
    {
        auto assetInfoFilePath = AZ::IO::Path{ sourceAssetPath };
        assetInfoFilePath.Native() += ".assetinfo";
        return CreateSceneManifest(sourceAssetPath, sourceAssetPath.Native() + ".assetinfo", collider, visual);
    }
} // namespace ROS2::Utils
