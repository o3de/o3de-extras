/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/IO/SystemFile.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/function/function_template.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/URDF/UrdfParser.h>

namespace ROS2
{
    namespace
    {
        static inline bool FileExistsCall(const AZStd::string& filename)
        {
            return AZ::IO::SystemFile::Exists(filename.c_str());
        };
    } // namespace

    namespace Utils
    {
        //! Determine whether a given link is likely a wheel link.
        //! This can be useful to provide a good default behavior - for example, to add Vehicle Dynamics components to this link's entity.
        //! @param link the link that will be subjected to the heuristic.
        //! @return true if the link is likely a wheel link.
        bool IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link);

        //! The recursive function for the given link goes through URDF and finds world-to-entity transformation for us.
        //! @param link pointer to URDF link that root of robot description
        //! @param t initial transform, should be identity for non-recursive call.
        //! @returns root to entity transform
        AZ::Transform GetWorldTransformURDF(const urdf::LinkSharedPtr& link, AZ::Transform t = AZ::Transform::Identity());

        //! Retrieve all links in URDF as a map, where a key is link's name and a value is a pointer to link.
        //! Allows to retrieve a pointer to a link given it name.
        //! @param childLinks list of links in a query
        //! @returns mapping from link name to link pointer
        AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> GetAllLinks(const std::vector<urdf::LinkSharedPtr>& childLinks);

        //! Retrieve all joints in URDF.
        //! @param childLinks list of links in a query
        //! @returns mapping from joint name to joint pointer
        AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> GetAllJoints(const std::vector<urdf::LinkSharedPtr>& childLinks);

        //! Retrieve all meshes referenced in URDF as unresolved URDF patches.
        //! Note that returned filenames are unresolved URDF patches.
        //! @param visual - search for visual meshes.
        //! @param colliders - search for collider meshes.
        //! @param rootLink - pointer to URDF link that is a root of robot description
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
            const AZStd::function<bool(const AZStd::string&)>& fileExists = FileExistsCall);

    } // namespace Utils
} // namespace ROS2