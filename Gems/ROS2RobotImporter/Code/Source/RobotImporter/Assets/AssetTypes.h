/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/IO/Path/Path.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/Math/Uuid.h>
#include <AzCore/base.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>

namespace ROS2RobotImporter::Assets
{
    //! Structure contains essential information about the source and product assets in O3DE.
    //! It is designed to provide necessary information for other classes of the importer, eg CollidersMaker or VisualsMaker.
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

    //! The structure contains a mapping between an unresolved URI from the source file and O3DE asset information.
    struct ReferencedAsset
    {
        ReferencedAsset() = default;

        //! The model URI associated with the asset.
        AZStd::string m_modelUri;

        //! Unresolved path to asset, eg `package://meshes/bar_link.dae`.
        AZ::IO::Path m_assetUri;

        //! Resolved path, points to the valid mesh in the filesystem, eg `/home/user/ros_ws/src/foo_robot/meshes/bar_link.dae'
        AZ::IO::Path m_resolvedPath;

        //! Checksum of the file pointed by `m_resolvedPath`.
        AZ::Crc32 m_resolvedFileCRC;

        //! Status of the copy process.
        CopyStatus m_copyStatus = Waiting;

        //! Type of asset reference(s) - mesh, texture, etc.
        ReferencedAssetType m_assetType;

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

    /// Type that hold result of mapping from asset name (model URI + asset URI) to asset info
    using ReferencedAssetMap = AZStd::unordered_map<AZ::IO::Path, Assets::ReferencedAsset>;
} // namespace ROS2RobotImporter::Assets
