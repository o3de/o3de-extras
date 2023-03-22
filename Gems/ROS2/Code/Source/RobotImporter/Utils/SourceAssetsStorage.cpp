/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "SourceAssetsStorage.h"
#include "RobotImporterUtils.h"

namespace ROS2::Utils
{
    /// Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZStd::string& filename)
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
        const AZStd::unordered_set<AZStd::string> kInterestingExtensions{ ".dae", ".stl", ".obj", ".fbx" };
        constexpr char AzModelExtension[] {".azmodel"} ;
        AZStd::unordered_map<AZ::Crc32, AvailableAsset> availableAssets;

        // take all meshes in catalog
        AZ::Data::AssetCatalogRequests::AssetEnumerationCB collectAssetsCb =
            [&](const AZ::Data::AssetId id, const AZ::Data::AssetInfo& info)
        {
            if (AZ::Data::AssetManager::Instance().GetHandler(info.m_assetType))
            {
                if (!info.m_relativePath.ends_with(AzModelExtension))
                {
                    return;
                }
                using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;
                bool pathFound{ false };
                AZStd::string fullSourcePathStr;

                AssetSysReqBus::BroadcastResult(
                    pathFound, &AssetSysReqBus::Events::GetFullSourcePathFromRelativeProductPath, info.m_relativePath, fullSourcePathStr);
                if (!pathFound)
                {
                    return;
                }
                const AZ::IO::Path fullSourcePath(fullSourcePathStr);
                const AZStd::string extension = fullSourcePath.Extension().Native();

                if (!kInterestingExtensions.contains(extension))
                {
                    return;
                }

                AZ::Crc32 crc = Utils::GetFileCRC(fullSourcePathStr);
                AZ_Printf(
                    "RobotImporterWidget",
                    "m_relativePath %s %s : %s %llu \n",
                    info.m_relativePath.c_str(),
                    info.m_assetId.ToString<AZStd::string>().c_str(),
                    fullSourcePath.c_str(),
                    crc);

                AvailableAsset foundAsset;
                foundAsset.m_sourceAssetRelativePath = info.m_relativePath;
                foundAsset.m_assetId = info.m_assetId;
                foundAsset.m_sourceAssetGlobalPath = fullSourcePathStr;
                foundAsset.m_productAssetRelativePath = info.m_relativePath;
                auto availableAssetIt = availableAssets.find(crc);
                if (availableAssetIt != availableAssets.end())
                {
                    const AZStd::string stem(fullSourcePath.Stem().Native());
                    // probably there is already submesh added. Replace only if there is exact name
                    if (info.m_relativePath.contains(stem + AzModelExtension))
                    {
                        availableAssetIt->second = foundAsset;
                    }
                }
                else
                {
                    availableAssets.insert({crc, foundAsset});
                }
            }
        };
        AZ::Data::AssetCatalogRequestBus::Broadcast(
            &AZ::Data::AssetCatalogRequestBus::Events::EnumerateAssets, nullptr, collectAssetsCb, nullptr);

        return availableAssets;
    }

    UrdfAssetMap FindAssetsForUrdf(const AZStd::unordered_set<AZStd::string>& meshesFilenames, const AZStd::string& urdFilename)
    {
        auto enviromentalVariable = std::getenv("AMENT_PREFIX_PATH");
        AZ_Warning("UrdfAssetMap", enviromentalVariable, "AMENT_PREFIX_PATH is not found.");
        AZStd::string amentPrefixPath {enviromentalVariable};

        UrdfAssetMap urdfToAsset;
        for (const auto& t : meshesFilenames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = t;
            asset.m_resolvedUrdfPath = Utils::ResolveURDFPath(asset.m_urdfPath, urdFilename, amentPrefixPath);
            asset.m_urdfFileCRC = Utils::GetFileCRC(asset.m_resolvedUrdfPath);
            urdfToAsset.emplace(t, AZStd::move(asset));
        }

        const AZStd::unordered_map<AZ::Crc32, AvailableAsset> availableAssets = Utils::GetInterestingSourceAssetsCRC();

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
        return urdfToAsset;
    }

} // namespace ROS2::Utils
