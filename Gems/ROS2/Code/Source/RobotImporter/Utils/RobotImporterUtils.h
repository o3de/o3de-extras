/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/ComponentBus.h"
#include "AzCore/std/string/string.h"
#include "RobotImporter/URDF/UrdfParser.h"
#include <AzCore/IO/SystemFile.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/function/function_template.h>

namespace ROS2
{
    namespace
    {
        const AZStd::function<bool(const AZStd::string&)> fileExistsCall = [](const AZStd::string& filename) -> bool
        {
            return AZ::IO::SystemFile::Exists(filename.c_str());
        };
    }

    namespace Utils
    {
        bool IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link);

        //! The recursive function for the given link goes through URDF and finds world-to-entity transformation for us.
        //! It traverses URDF from the given link to the root.
        //! @param t - should be default (identity).
        //! @returns root to entity transform
        AZ::Transform GetWorldTransformURDF(const urdf::LinkSharedPtr& link, AZ::Transform t = AZ::Transform::Identity());

        //! Retrieve all child links in urdf.
        //! @param child links list of links in a query
        //! @returns mapping from link name to link pointer
        AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> GetAllLinks(const std::vector<urdf::LinkSharedPtr>& childLinks);

        //! Retrieve all joints in URDF.
        //! @param child links list of links in a query
        //! @returns mapping from link name to link pointer
        AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> GetAllJoints(const std::vector<urdf::LinkSharedPtr>& childLinks);

        //! Retrieve all meshes referenced in URDF as unresolved URDF patches.
        //! Function traverse URDF in recursive manner.
        //! It obtains referenced meshes' filenames.
        //! Note that returned filenames are unresolved URDF patches.
        //! @param visual - search for visual meshes.
        //! @param colliders - search for collider meshes.
        //! @returns set of meshes' filenames.
        AZStd::unordered_set<AZStd::string> GetMeshesFilenames(const urdf::LinkConstSharedPtr& rootLink, bool visual, bool colliders);

        //! Resolves path from unresolved URDF path.
        //! @param unresolvedPath - unresolved URDF path, example : `package://meshes/foo.dae`.
        //! @param urdfFilePath - the absolute path of URDF file which contains the path that is to be resolved.
        //! @param fileExists - functor to check if the given file exists. Exposed for unit test, default one should be used.
        //! @returns resolved path to the mesh
        AZStd::string ResolveURDFPath(
            AZStd::string unresolvedPath,
            const AZStd::string& urdfFilePath,
            const AZStd::function<bool(const AZStd::string&)>& fileExists = fileExistsCall);

    } // namespace Utils
} // namespace ROS2