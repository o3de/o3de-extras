/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzFramework/Asset/AssetSystemBus.h>
#include <RobotImporter/Assets/AssetTypes.h>

namespace ROS2RobotImporter
{
    struct SdfAssetBuilderSettings;
} // namespace ROS2RobotImporter

namespace ROS2RobotImporter::Assets
{

    using CopyAssetsStatusCallback =
        AZStd::function<void(Assets::CopyStatus copyStatus, const AZ::IO::Path& filename, const ReferencedAsset& asset)>;

    //! Copies and prepares assets that are referenced in SDF/URDF.
    //! It resolves every asset, creates a directory in Project's Asset directory, copies files, and prepares assets info.
    //! Finally, it assembles its results into mapping that allows mapping the SDF/URDF mesh name to the source asset.
    //! @param referencedAssetMap - files to copy (as unresolved URIs)
    //! @param sourceFilePath - path to the SDF/URDF file (as a global path)
    //! @param outputDirSuffix - suffix to make output directory unique, if xacro file was used
    //! @param fileIO - instance to fileIO class
    bool CopyReferencedAssets(
        ReferencedAssetMap& referencedAssetMap,
        const AZ::IO::Path& sourceFilePath,
        CopyAssetsStatusCallback statusCallback = {},
        AZStd::string_view outputDirSuffix = "",
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Copies and prepares asset that is referenced in SDF/URDF.
    //! Modifies referencedAsset in place.
    //! @param importedAssetsDest - destination ImportedAssetsDest for imported assets.
    //! @param referencedAsset - asset info. Will be modified.
    //! @param duplicationCounter - number indication the number of times the asset has been duplicated
    //! @param fileIO - instance to fileIO class
    //! @returns status of the copy process
    CopyStatus CopyReferencedAsset(
        const ImportedAssetsDest& importedAssetsDest,
        Assets::ReferencedAsset& referencedAsset,
        unsigned int duplicationCounter,
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Prepares temporary and final directory for imported assets.
    //! @param sourceFilePath - path to the SDF/URDF file (as a global path)
    //! @param outputDirSuffix - name of the output directory
    //! @param fileIO - instance to fileIO class
    //! @returns structure containing paths to temporary and final directory for imported assets, or failure if failed to create.
    AZ::Outcome<ImportedAssetsDest> PrepareImportedAssetsDest(
        const AZ::IO::Path& sourceFilePath,
        AZStd::string_view outputDirSuffix = "",
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Remove temporary directory for imported assets.
    //! @param tmpDir - path to temporary directory
    //! @param fileIO - instance to fileIO class
    //! @returns true if succeed
    AZ::Outcome<bool> RemoveTmpDir(const AZ::IO::Path $tmpDir, AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Flushes the IO of an asset to disk
    //! @param path - path to asset to flush
    //! @returns status of the asset after flushing
    AzFramework::AssetSystem::AssetStatus FlushIOOfAsset(const AZ::IO::Path& path);
} // namespace ROS2RobotImporter::Assets