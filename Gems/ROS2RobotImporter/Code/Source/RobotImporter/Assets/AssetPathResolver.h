/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/function/function_template.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/Assets/AssetTypes.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

#include <sdf/Root.hh>

namespace ROS2RobotImporter::Utils
{
    inline namespace Internal
    {
        bool FileExistsCall(const AZ::IO::PathView& filePath);
    } // namespace Internal
} // namespace ROS2RobotImporter::Utils

namespace ROS2RobotImporter::Utils
{
    //! Retrieve all assets referenced in SDF/URDF as unresolved URIs.
    //! The URIs will still need to get resolved via ResolveAssetPath() to point to a valid file location.
    //! @param root reference to SDF Root object representing the root of the parsed SDF xml document
    //! @returns mapping from asset name (model URI + asset URI) to asset info.
    ReferencedAssetMap GetReferencedAssetFilenames(const sdf::Root& root);

    //! Callback used to check for file exist of a path referenced within a URDF/SDF file
    //! @param path Candidate local filesystem path to check for existence
    //! @return true should be returned if the file exist otherwise false
    using FileExistsCB = AZStd::function<bool(const AZ::IO::PathView&)>;

    //! Resolves path for an asset referenced in a URDF/SDF file.
    //! @param unresolvedPath unresolved URDF/SDF path, example : `model://meshes/foo.dae`.
    //! @param baseFilePath the absolute path of URDF/SDF file which contains the path that is to be resolved.
    //! @param amentPrefixPath the string that contains available packages' path, separated by ':' signs.
    //! @param settings the asset path resolution settings to use for attempting to locate the correct files
    //! @param fileExists functor to check if the given file exists. Exposed for unit test, default one should be used.
    //! @returns resolved path to the referenced file within the URDF/SDF, or the passed-in path if no resolution was possible.
    AZ::IO::Path ResolveAssetPath(
        AZ::IO::Path unresolvedPath,
        const AZ::IO::PathView& baseFilePath,
        AZStd::string_view amentPrefixPath,
        const SdfAssetBuilderSettings& settings,
        const FileExistsCB& fileExists = &Internal::FileExistsCall);

    using AmentPrefixString = AZStd::fixed_string<32768>;
    AmentPrefixString GetAmentPrefixPath();
} // namespace ROS2RobotImporter::Utils
