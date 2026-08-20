/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotImporter/Assets/AssetLookup.h>

#include <AssetDatabase/AssetDatabaseConnection.h>
#include <Atom/RPI.Reflect/Image/StreamingImageAsset.h>
#include <Atom/RPI.Reflect/Model/ModelAsset.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/IO/SystemFile.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/algorithm.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <PhysX/MeshAsset.h>
#include <RobotImporter/Assets/AssetPathResolver.h>
#include <SceneAPI/SceneCore/Containers/Scene.h>
#include <SceneAPI/SceneCore/Containers/Utilities/Filters.h>
#include <SceneAPI/SceneCore/DataTypes/GraphData/IMaterialData.h>
#include <SceneAPI/SceneCore/Events/SceneSerializationBus.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

namespace ROS2RobotImporter::Assets
{

    namespace
    {
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

            Visitor assetImporterVisitor;
            settingsRegistry->Visit(
                assetImporterVisitor,
                AZ::SettingsRegistryInterface::FixedValueString("/O3DE/SceneAPI/AssetImporter/SupportedFileTypeExtensions"));
            return assetImporterVisitor.m_supportedFileExtensions;
        }
    } // namespace

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

            AZ::Crc32 crc = Assets::GetFileCRC(foundAsset.m_sourceAssetGlobalPath);
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

    void FindReferencedAssets(ReferencedAssetMap& referencedAssetMap)
    {
        if (referencedAssetMap.empty())
        {
            return;
        }

        // Try to match assets by exact path first. For unmatched assets look for a copy with the same content.
        AZStd::vector<ReferencedAsset*> unmatchedAssets;
        for (auto& [unresolvedFileName, asset] : referencedAssetMap)
        {
            if (asset.m_resolvedPath.empty())
            {
                continue;
            }

            asset.m_resolvedFileCRC = Assets::GetFileCRC(asset.m_resolvedPath);
            asset.m_availableAssetInfo = Assets::GetAvailableAssetInfo(asset.m_resolvedPath);
            if (asset.m_availableAssetInfo.m_sourceGuid.IsNull())
            {
                unmatchedAssets.emplace_back(&asset);
            }
        }

        if (unmatchedAssets.empty())
        {
            return;
        }

        AZStd::unordered_map<AZ::Crc32, AvailableAsset> availableAssets = Assets::GetInterestingSourceAssetsCRC();

        // Search for suitable mappings by comparing checksum
        for (auto& unmatchedAsset : unmatchedAssets)
        {
            if (auto foundSourceAsset = availableAssets.find(unmatchedAsset->m_resolvedFileCRC); foundSourceAsset != availableAssets.end())
            {
                unmatchedAsset->m_availableAssetInfo = foundSourceAsset->second;
            }
        }
    }

    AvailableAsset GetAvailableAssetInfo(const AZ::IO::Path& globalSourceAssetPath)
    {
        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;

        bool sourceAssetFound{ false };
        AZ::Data::AssetInfo assetInfo;
        AZStd::string watchFolder;
        AvailableAsset foundAsset;

        // get source asset info
        AssetSysReqBus::BroadcastResult(
            sourceAssetFound, &AssetSysReqBus::Events::GetSourceInfoBySourcePath, globalSourceAssetPath.c_str(), assetInfo, watchFolder);

        if (!sourceAssetFound)
        {
            // Not an error on its own - the file may be outside every scan folder, or not scanned yet.
            AZ_Trace("GetAvailableAssetInfo", "Cannot find source asset info for %s\n", globalSourceAssetPath.c_str());
            return foundAsset;
        }

        const auto fullSourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(assetInfo.m_relativePath);
        foundAsset.m_sourceAssetRelativePath = assetInfo.m_relativePath;
        foundAsset.m_sourceAssetGlobalPath = fullSourcePath.String();
        foundAsset.m_sourceGuid = assetInfo.m_assetId.m_guid;

        AZ::Crc32 crc = Assets::GetFileCRC(foundAsset.m_sourceAssetGlobalPath);
        if (crc == AZ::Crc32(0))
        {
            AZ_Warning("GetInterestingSourceAssetsCRC", false, "Zero CRC for source asset %s", foundAsset.m_sourceAssetGlobalPath.c_str());
            return foundAsset;
        }

        return foundAsset;
    }

    void ResolveAssetMap(
        ReferencedAssetMap& unresolvedAssetMap, const AZ::IO::Path& sourceFilePath, const SdfAssetBuilderSettings& sdfBuilderSettings)
    {
        auto amentPrefixPath = Assets::GetAmentPrefixPath();

        for (auto& [unresolvedFileName, asset] : unresolvedAssetMap)
        {
            asset.m_resolvedPath = Assets::ResolveAssetPath(unresolvedFileName, sourceFilePath, amentPrefixPath, sdfBuilderSettings);
            asset.m_resolvedFileCRC = AZ::Crc32();
        }
    }

    AZStd::vector<AZStd::string> GetProductAssets(const AZ::Uuid& sourceAssetUUID)
    {
        AZStd::vector<AZStd::string> productPaths;
        const auto importantTypes = AZStd::to_array<const AZ::TypeId>({ azrtti_typeid<AZ::RPI::StreamingImageAsset>(),
                                                                        azrtti_typeid<AZ::RPI::ModelAsset>(),
                                                                        azrtti_typeid<PhysX::Pipeline::MeshAsset>() });

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

} // namespace ROS2RobotImporter::Assets