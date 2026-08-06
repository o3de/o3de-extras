/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/Assets/AssetTypes.h>

namespace ROS2RobotImporter::Utils
{
    //! Retrieves the O3DE source asset info for an already existing source asset.
    //! @param globalSourceAssetPath - global path to the source asset
    //! @returns found asset info, or a default constructed one when the source asset is not known to the Asset Processor
    AvailableAsset GetAvailableAssetInfo(const AZStd::string& globalSourceAssetPath);

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

} // namespace ROS2RobotImporter::Utils
