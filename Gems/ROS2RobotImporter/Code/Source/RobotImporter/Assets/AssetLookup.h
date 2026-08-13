/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/Math/Uuid.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/Assets/AssetTypes.h>

namespace ROS2RobotImporter
{
    struct SdfAssetBuilderSettings;
} // namespace ROS2RobotImporter

namespace ROS2RobotImporter::Assets
{
    //! Function computes CRC32 on first kilobyte of file.
    AZ::Crc32 GetFileCRC(const AZ::IO::Path& filename);

    //! Compute CRC for every source mesh from the assets catalog.
    //! @returns map where key is crc of source file and value is AvailableAsset.
    AZStd::unordered_map<AZ::Crc32, AvailableAsset> GetInterestingSourceAssetsCRC();

    //! Discover an association between meshes in input SDF/URDF and O3DE source and product assets.
    //! Requires the resolved paths to be filled in beforehand by `ResolveAssetMap`.
    //! Steps:
    //! - Assets resolved to a path inside an Asset Processor scan folder are looked up by that path `LookupSourceAssetByPath`.
    //! - Files pointed by the remaining resolved paths have their checksum computed `GetFileCRC`.
    //! - Suitable mapping to the O3DE asset is found by comparing the checksum of the file pointed by the resolved path and source asset
    //! @param referencedAssetMap - list of the assets from the SDF/URDF file that are to be found as O3DE assets
    void FindReferencedAssets(ReferencedAssetMap& referencedAssetMap);

    //! Creates a mapping from unresolved URIs to source asset info.
    //! @param unresolvedAssetMap - list of assets discovered in the input SDF/URDF file
    //! @param sourceFilePath - path to the SDF/URDF file (as a global path)
    //! @param sdfBuilderSettings - the builder settings to use to convert the SDF/URDF files
    void ResolveAssetMap(
        ReferencedAssetMap& unresolvedAssetMap, const AZ::IO::Path& sourceFilePath, const SdfAssetBuilderSettings& sdfBuilderSettings);

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

    //! Creates a list of files referenced in an asset (e.g. materials)
    //! @param sourceMeshAssetPath - global path to source asset used to find scene
    //! @returns list of file paths referenced in the scene
    AZStd::unordered_set<AZ::IO::Path> GetMeshTextureAssets(const AZ::IO::Path& sourceMeshAssetPath);

    //! Retrieves the O3DE source asset info for an already existing source asset.
    //! @param globalSourceAssetPath - global path to the source asset
    //! @returns found asset info, or a default constructed one when the source asset is not known to the Asset Processor
    AvailableAsset GetAvailableAssetInfo(const AZ::IO::Path& sourceAssetGlobalPath);

} // namespace ROS2RobotImporter::Assets