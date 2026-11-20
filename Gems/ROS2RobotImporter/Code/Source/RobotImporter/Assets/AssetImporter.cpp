/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AssetImporter.h"

#include <AzCore/Math/Crc.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/parallel/mutex.h>
#include <RobotImporter/Assets/AssetLookup.h>
#include <RobotImporter/Assets/SceneManifestBuilder.h>

namespace ROS2RobotImporter::Assets
{
    bool CopyReferencedAssets(
        ReferencedAssetMap& referencedAssetMap,
        const AZ::IO::Path& sourceFilePath,
        CopyAssetsStatusCallback statusCallback,
        AZStd::string_view outputDirSuffix,
        AZ::IO::FileIOBase* fileIO)
    {
        if (referencedAssetMap.empty())
        {
            return true;
        }

        auto destDirectory = PrepareImportedAssetsDest(sourceFilePath, outputDirSuffix, fileIO);
        if (!destDirectory.IsSuccess())
        {
            AZ_Error("CopyReferencedAsset", false, "Failed to create destination folder for imported assets");
            return false;
        }

        AZStd::unordered_map<AZ::IO::Path, unsigned int> duplicatedFilenames;
        for (auto& [unresolvedFileName, referencedAsset] : referencedAssetMap)
        {
            auto copyStatus = CopyStatus::Copying;
            if (duplicatedFilenames.contains(referencedAsset.m_assetUri))
            {
                duplicatedFilenames[referencedAsset.m_assetUri]++;
            }
            else
            {
                duplicatedFilenames[referencedAsset.m_assetUri] = 0;
            }
            if (statusCallback)
            {
                statusCallback(copyStatus, unresolvedFileName, referencedAsset);
            }
            copyStatus = CopyStatus::Unresolvable;
            if (referencedAsset.m_resolvedPath.empty())
            {
                AZ_Warning("CopyReferencedAsset", false, "There is no resolved path for %s", unresolvedFileName.c_str());
            }
            else
            {
                copyStatus =
                    CopyReferencedAsset(destDirectory.GetValue(), referencedAsset, duplicatedFilenames[referencedAsset.m_assetUri]);
            }
            if (statusCallback)
            {
                statusCallback(copyStatus, unresolvedFileName, referencedAsset);
            }
        }
        RemoveTmpDir(destDirectory.GetValue().importDirectoryTmp);

        return true;
    }

    AZ::Outcome<ImportedAssetsDest> PrepareImportedAssetsDest(
        const AZ::IO::Path& sourceFilePath, AZStd::string_view outputDirSuffix, AZ::IO::FileIOBase* fileIO)
    {
        AZ_Assert(fileIO, "No FileIO instance");
        AZ::Crc32 sourceFileCrc;
        sourceFileCrc.Add(sourceFilePath.c_str(), sourceFilePath.Native().size());

        // By naming the temp directory '$tmp_*', the default configuration in AssetProcessorPlatformConfig.setreg will
        // exclude these files from processing.
        const AZStd::string directoryNameTmp = AZStd::string::format("$tmp_%u.tmp", AZ::u32(sourceFileCrc));
        const auto directoryNameDst = AZ::IO::FixedMaxPathString::format(
            "%u_%.*s%.*s", AZ::u32(sourceFileCrc), AZ_PATH_ARG(sourceFilePath.Stem()), AZ_STRING_ARG(outputDirSuffix));

        const AZ::IO::Path importDirectoryTmp = AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / directoryNameTmp;
        const AZ::IO::Path importDirectoryDst = AZ::IO::Path(AZ::Utils::GetProjectPath()) / "Assets" / "Importer" / directoryNameDst;

        fileIO->DestroyPath(importDirectoryTmp.c_str());
        const auto outcomeCreateDstDir = fileIO->CreatePath(importDirectoryDst.c_str());
        const auto outcomeCreateTmpDir = fileIO->CreatePath(importDirectoryTmp.c_str());
        AZ_Error("CopyReferencedAsset", outcomeCreateDstDir, "Cannot create destination directory : %s", importDirectoryDst.c_str());
        AZ_Error("CopyReferencedAsset", outcomeCreateTmpDir, "Cannot create temporary directory : %s", importDirectoryTmp.c_str());

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
            AZ_Error("CopyReferencedAsset", false, "Cannot remove temporary directory : %s", $tmpDir.c_str());
            return AZ::Failure();
        }
        return AZ::Success(true);
    }

    CopyStatus CopyReferencedAsset(
        const ImportedAssetsDest& importedAssetsDest,
        Assets::ReferencedAsset& referencedAsset,
        unsigned int duplicationCounter,
        AZ::IO::FileIOBase* fileIO)
    {
        AZStd::string filename = referencedAsset.m_resolvedPath.Filename().String();
        if (duplicationCounter > 0)
        {
            AZStd::string stem = referencedAsset.m_resolvedPath.Stem().String();
            AZStd::string extension = referencedAsset.m_resolvedPath.Extension().String();
            filename = AZStd::string::format("%s_%u%s", stem.c_str(), duplicationCounter, extension.c_str());
        }

        AZ::IO::Path targetPathAssetDst(importedAssetsDest.importDirectoryDst / filename);
        AZ::IO::Path targetPathAssetTmp(importedAssetsDest.importDirectoryTmp / filename);

        AZ::IO::Path targetPathAssetInfo(targetPathAssetDst.Native() + ".assetinfo");

        bool targetAssetExists = fileIO->Exists(targetPathAssetDst.c_str());
        if (!targetAssetExists)
        {
            referencedAsset.m_copyStatus = CopyStatus::Copying;

            // copy mesh file to temporary location ignored by AP
            const auto outcomeCopyTmp = fileIO->Copy(referencedAsset.m_resolvedPath.c_str(), targetPathAssetTmp.c_str());
            AZ_Printf(
                "CopyReferencedAsset",
                "Copy %s to %s, result: %d",
                referencedAsset.m_resolvedPath.c_str(),
                targetPathAssetTmp.c_str(),
                outcomeCopyTmp.GetResultCode());

            if (outcomeCopyTmp)
            {
                // call FlushIOOfAsset to ensure the asset processor is aware of the new file
                FlushIOOfAsset(targetPathAssetTmp);

                const bool needsVisual = (referencedAsset.m_assetType & ReferencedAssetType::VisualMesh) == ReferencedAssetType::VisualMesh;
                const bool needsCollider =
                    (referencedAsset.m_assetType & ReferencedAssetType::ColliderMesh) == ReferencedAssetType::ColliderMesh;
                const bool isMeshFile = (needsVisual || needsCollider);

                // if the asset is a mesh, create asset info at destination location using the temporary mesh file
                const bool assetInfoOk =
                    isMeshFile ? CreateSceneManifest(targetPathAssetTmp, targetPathAssetInfo, needsCollider, needsVisual) : true;

                if (assetInfoOk)
                {
                    // copy additional assets such as textures directly to destination location
                    if (isMeshFile)
                    {
                        const auto& meshTextureAssets = Assets::GetMeshTextureAssets(targetPathAssetTmp);
                        for (const auto& unresolvedAssetPath : meshTextureAssets)
                        {
                            // Manifest returns local path in Project's directory temp folder
                            const AZ::IO::Path assetLocalPath(AZ::IO::Path(AZ::IO::Path(AZ::Utils::GetProjectPath()) / unresolvedAssetPath)
                                                                  .LexicallyRelative(importedAssetsDest.importDirectoryTmp));

                            const AZ::IO::Path assetFullPathSrc(AZ::IO::Path(referencedAsset.m_resolvedPath.ParentPath()) / assetLocalPath);
                            const AZ::IO::Path assetFullPathDst(importedAssetsDest.importDirectoryDst / assetLocalPath);

                            const auto outcomeMkdir = fileIO->CreatePath(AZ::IO::Path(assetFullPathDst.ParentPath()).c_str());
                            if (!outcomeMkdir)
                            {
                                break;
                            }

                            const auto outcomeCopy = fileIO->Copy(assetFullPathSrc.c_str(), assetFullPathDst.c_str());
                            if (!outcomeCopy)
                            {
                                referencedAsset.m_copyStatus = CopyStatus::Failed;
                            }
                        }
                    }

                    // move asset file from temporary location to destination location
                    const auto outcomeMoveDst = fileIO->Rename(targetPathAssetTmp.c_str(), targetPathAssetDst.c_str());
                    AZ_Printf(
                        "CopyReferencedAsset",
                        "Rename file %s to %s, result: %d",
                        targetPathAssetTmp.c_str(),
                        targetPathAssetDst.c_str(),
                        outcomeMoveDst.GetResultCode());

                    // call FlushIOOfAsset to ensure the asset processor is aware of the new file
                    FlushIOOfAsset(targetPathAssetDst);

                    referencedAsset.m_copyStatus = outcomeMoveDst ? CopyStatus::Copied : CopyStatus::Failed;
                }
            }
            else
            {
                referencedAsset.m_copyStatus = CopyStatus::Failed;
            }
        }
        else
        {
            AZ_Printf("CopyReferencedAsset", "File %s already exists, omitting import", targetPathAssetDst.c_str());
            referencedAsset.m_copyStatus = CopyStatus::Exists;
        }

        if (referencedAsset.m_copyStatus == CopyStatus::Exists || referencedAsset.m_copyStatus == CopyStatus::Copied)
        {
            referencedAsset.m_availableAssetInfo = Assets::GetAvailableAssetInfo(targetPathAssetDst.String());
        }
        referencedAsset.m_resolvedFileCRC = AZ::Crc32();

        return referencedAsset.m_copyStatus;
    }

    AzFramework::AssetSystem::AssetStatus FlushIOOfAsset(const AZ::IO::Path& path)
    {
        AzFramework::AssetSystem::AssetStatus assetStatus = AzFramework::AssetSystem::AssetStatus::AssetStatus_Unknown;
        AzFramework::AssetSystemRequestBus::BroadcastResult(
            assetStatus, &AzFramework::AssetSystem::AssetSystemRequests::GetAssetStatus_FlushIO, path.c_str());
        AZ_Warning(
            "CopyReferencedAsset",
            assetStatus != AzFramework::AssetSystem::AssetStatus::AssetStatus_Unknown,
            "Asset processor did not recognize the new file %s.",
            path.c_str());

        return assetStatus;
    }
} // namespace ROS2RobotImporter::Assets