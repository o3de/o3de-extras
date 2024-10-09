/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AssetDatabase/AssetDatabaseConnection.h>
#include <AssetDatabase/PathOrUuid.h>
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzFramework/Asset/AssetSystemBus.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>

namespace ROS2
{
    struct SdfAssetBuilderSettings;
} // namespace ROS2

namespace ROS2::Utils
{
    //! Structure contains essential information about the source and product assets in O3DE.
    //! It is designed to provide necessary information for other classes in URDF converter, eg CollidersMaker or VisualsMaker.
    struct AvailableAsset
    {
        //! Relative path to source asset eg `Assets/foo_robot/meshes/bar_link.dae`.
        AZ::IO::Path m_sourceAssetRelativePath;

        //! Relative path to source asset eg `/home/user/project/Assets/foo_robot/meshes/bar_link.dae`.
        AZ::IO::Path m_sourceAssetGlobalPath;

        //! Relative path to source asset eg `foo_robot/meshes/bar_link.azmodel`.
        AZ::IO::Path m_productAssetRelativePath;

        //! Source GUID of source asset
        AZ::Uuid m_sourceGuid = AZ::Uuid::CreateNull();
    };

    //! Bitfield containing the types of asset references are associated with a given unresolved URI or path reference.
    //! These are flags because the same mesh URI can refer to both a Visual and a Collider entry, for example.
    enum class ReferencedAssetType
    {
        VisualMesh = 0b00000001, //! URI references a mesh for a Visual entry
        ColliderMesh = 0b00000010, //! URI references a mesh for a Collider entry
        Texture = 0b00000100, //! URI references one of a multitude of texture types (Diffuse, Normal, AO, etc)
    };
    AZ_DEFINE_ENUM_BITWISE_OPERATORS(ReferencedAssetType);

    //! Status of the copy process.
    enum CopyStatus
    {
        Unresolvable, //! Unresolvable
        Waiting, //! Waiting for copy
        Copying, //! Copying
        Copied, //! Copied
        Exists, //! Already exists
        Failed, //! Failed
    };

    //! The structure contains a mapping between URDF's path to O3DE asset information.
    struct UrdfAsset
    {
        //! Unresolved URDF path to mesh, eg `package://meshes/bar_link.dae`.
        AZ::IO::Path m_urdfPath;

        //! Resolved URDF path, points to the valid mesh in the filesystem, eg `/home/user/ros_ws/src/foo_robot/meshes/bar_link.dae'
        AZ::IO::Path m_resolvedUrdfPath;

        //! Unresolved file name, points to the valid mesh in the filesystem
        AZStd::string m_unresolvedFileName;

        //! Checksum of the file located pointed by `m_resolvedUrdfPath`.
        AZ::Crc32 m_urdfFileCRC;

        //! Status of the copy process.
        CopyStatus m_copyStatus = Waiting;

        //! Type of asset reference(s) - mesh, texture, etc.
        ReferencedAssetType m_assetReferenceType;

        //! Found O3DE asset.
        AvailableAsset m_availableAssetInfo;
    };

    //! Structure contains paths to the temporary and destination directories for imported assets.
    struct ImportedAssetsDest
    {
        //! Temporary directory for imported assets.
        AZ::IO::Path importDirectoryTmp;

        //! Destination directory for imported assets.
        AZ::IO::Path importDirectoryDst;
    };

    //! Maps unresolved URI asset references to the type of reference(s) - mesh, texture, etc.
    using AssetFilenameReferences = AZStd::unordered_map<AZStd::string, ReferencedAssetType>;

    /// Type that hold result of mapping from URDF path to asset info
    using UrdfAssetMap = AZStd::unordered_map<AZ::IO::Path, Utils::UrdfAsset>;

    //! Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZ::IO::Path& filename);

    //! Compute CRC for every source mesh from the assets catalog.
    //! @returns map where key is crc of source file and value is AvailableAsset.
    AZStd::unordered_map<AZ::Crc32, AvailableAsset> GetInterestingSourceAssetsCRC();

    //! Discover an association between meshes in URDF and O3DE source and product assets.
    //! The @param meshesFilenames contains the list of unresolved URDF filenames that are to be found as assets.
    //! Steps:
    //! - Functions resolves URDF filenames with `ResolveAssetPath`.
    //! - Files pointed by resolved URDF patches have their checksum computed `GetFileCRC`.
    //! - Function scans all available O3DE assets by calling `GetInterestingSourceAssetsCRC`.
    //! - Suitable mapping to the O3DE asset is found by comparing the checksum of the file pointed by the URDF path and source asset.
    //! @param assetFilenames - list of the unresolved paths from the SDF/URDF file
    //! @param urdfFilename - filename of URDF file, used for resolvement
    //! @param sdfBuilderSettings - the builder settings that should be used to resolve paths
    //! @returns a URDF Asset map where the key is unresolved URDF path to AvailableAsset
    UrdfAssetMap FindReferencedAssets(
        const AssetFilenameReferences& assetFilenames,
        const AZStd::string& urdfFilename,
        const SdfAssetBuilderSettings& sdfBuilderSettings);

    //! Helper function that gets all the potential primary product asset paths from the source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @returns vector of relative paths to products, empty if no products are found
    AZStd::vector<AZStd::string> GetProductAssets(const AZ::Uuid& sourceAssetUUID);

    //! Helper function that gives the desired product asset ID from source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @param typeId type of product asset
    //! @returns product asset id (invalid id if not found)
    AZ::Data::AssetId GetProductAssetId(const AZ::Uuid& sourceAssetUUID, const AZ::TypeId typeId);

    //! Helper function that gives AZ::RPI::ImageAsset product asset ID from source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @returns product asset id (invalid id if not found)
    AZ::Data::AssetId GetImageProductAssetId(const AZ::Uuid& sourceAssetUUID);

    //! Helper function that gives AZ::RPI::ModelAsset product asset ID from source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @returns product asset id (invalid id if not found)
    AZ::Data::AssetId GetModelProductAssetId(const AZ::Uuid& sourceAssetUUID);

    //! Helper function that gives PhysX::Pipeline::MeshAsset product asset ID from source asset GUID
    //! @param sourceAssetUUID is source asset GUID
    //! @returns product asset id (invalid id if not found)
    AZ::Data::AssetId GetPhysXMeshProductAssetId(const AZ::Uuid& sourceAssetUUID);

    //! Creates side-car file (.assetinfo) that configures the imported scene (e.g. DAE file).
    //! The .assetinfo will be create next to scene's file.
    //! @param sourceAssetPath - global path to source asset
    //! @param collider - create assetinfo section for collider product asset
    //! @param visual - create assetinfo section for visual mesh
    //! @returns true if succeed
    bool CreateSceneManifest(const AZ::IO::Path& sourceAssetPath, const bool collider, const bool visual);

    //! Creates side-car file (.assetinfo) that configures the imported scene (e.g. DAE file).
    //! @param sourceAssetPath - global path to source asset
    //! @param assetInfoFile - global path to assetInfo file to create
    //! @param collider - create assetinfo section for collider product asset
    //! @param visual - create assetinfo section for visual mesh
    //! @returns true if succeed
    bool CreateSceneManifest(
        const AZ::IO::Path& sourceAssetPath, const AZ::IO::Path& assetInfoFile, const bool collider, const bool visual);

    //! Copies and prepares assets that are referenced in SDF/URDF.
    //! It resolves every asset, creates a directory in Project's Asset directory, copies files, and prepares assets info.
    //! Finally, it assembles its results into mapping that allows mapping the SDF/URDF mesh name to the source asset.
    //! @param assetFilenames - files to copy (as unresolved urdf paths)
    //! @param urdfFilename - path to URDF file (as a global path)
    //! @param sdfBuilderSettings - the builder settings to use to convert the SDF/URDF files
    //! @param outputDirSuffix - suffix to make output directory unique, if xacro file was used
    //! @param fileIO - instance to fileIO class
    //! @returns mapping from unresolved urdf paths to source asset info
    UrdfAssetMap CopyReferencedAssetsAndCreateAssetMap(
        const AssetFilenameReferences& assetFilenames,
        const AZStd::string& urdfFilename,
        const SdfAssetBuilderSettings& sdfBuilderSettings,
        AZStd::string_view outputDirSuffix = "",
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Creates a mapping from unresolved URDF paths to source asset info.
    //! @param assetFilenames - files to copy (as unresolved urdf paths)
    //! @param urdfFilename - path to URDF file (as a global path)
    //! @param sdfBuilderSettings - the builder settings to use to convert the SDF/URDF files
    //! @returns mapping from unresolved urdf paths to source asset info
    UrdfAssetMap CreateAssetMap(
        const AssetFilenameReferences& assetFilenames,
        const AZStd::string& urdfFilename,
        const SdfAssetBuilderSettings& sdfBuilderSettings);

    //! Copies and prepares asset that is referenced in SDF/URDF.
    //! Modifies urdfAsset in place.
    //! @param unresolvedFileName - unresolved urdf path to asset
    //! @param importedAssetsDest - destination ImportedAssetsDest for imported assets.
    //! @param urdfAsset - asset info. Will be modified.
    //! @param duplicationCounter - number indication the number of times the asset has been duplicated
    //! @param fileIO - instance to fileIO class
    //! @returns status of the copy process
    CopyStatus CopyReferencedAsset(
        const AZ::IO::Path& unresolvedFileName,
        const ImportedAssetsDest& importedAssetsDest,
        Utils::UrdfAsset& urdfAsset,
        unsigned int duplicationCounter,
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Prepares temporary and final directory for imported assets.
    //! @param urdfFilename - path to URDF file (as a global path)
    //! @param outputDirSuffix - name of the output directory
    //! @param fileIO - instance to fileIO class
    //! @returns structure containing paths to temporary and final directory for imported assets, or failure if failed to create.
    AZ::Outcome<ImportedAssetsDest> PrepareImportedAssetsDest(
        const AZStd::string& urdfFilename,
        AZStd::string_view outputDirSuffix = "",
        AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Remove temporary directory for imported assets.
    //! @param tmpDir - path to temporary directory
    //! @param fileIO - instance to fileIO class
    //! @returns true if succeed
    AZ::Outcome<bool> RemoveTmpDir(const AZ::IO::Path $tmpDir, AZ::IO::FileIOBase* fileIO = AZ::IO::FileIOBase::GetInstance());

    //! Creates a list of files referenced in an asset (e.g. materials)
    //! @param sourceMeshAssetPath - global path to source asset used to find scene
    //! @returns list of file paths referenced in the scene
    AZStd::unordered_set<AZ::IO::Path> GetMeshTextureAssets(const AZ::IO::Path& sourceMeshAssetPath);

    //! Flushes the IO of an asset to disk
    //! @param path - path to asset to flush
    //! @returns status of the asset after flushing
    AzFramework::AssetSystem::AssetStatus FlushIOOfAsset(const AZ::IO::Path& path);

} // namespace ROS2::Utils
