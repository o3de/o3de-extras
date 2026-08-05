/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/string/string_view.h>

#include <sdf/sdf.hh>

namespace ROS2RobotImporter::Utils
{
    //! Retrieve all links in URDF/SDF as a map, where a key is link's fully qualified name and a value is a pointer to link.
    //! Allows to retrieve a pointer to a link given it name.
    //! @param sdfModel object of SDF document corresponding to the <model> tag. It used to query links
    //! @param gatherNestedModelLinks When true recurses to any nested <model> tags of the Model object and also gathers their links as well
    //! @returns mapping from fully qualified link name(such as "model_name::link_name") to link pointer
    AZStd::unordered_map<AZStd::string, const sdf::Link*> GetAllLinks(const sdf::Model& sdfModel, bool gatherNestedModelLinks = false);

    //! Retrieve all joints in URDF/SDF where the key is the full composed path following the name scoping proposal in SDF 1.8
    //! http://sdformat.org/tutorials?tut=composition_proposal#1-nesting-and-encapsulation
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as
    //! well
    //! @returns mapping from fully qualified joint name(such as "model_name::joint_name") to joint pointer
    AZStd::unordered_map<AZStd::string, const sdf::Joint*> GetAllJoints(const sdf::Model& sdfModel, bool gatherNestedModelJoints = false);

    //! Retrieve all joints from URDF/SDF in which the specified link is a child in a sdf::Joint.
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It is used to query joints
    //! @param linkName Name of link which to query in joint objects ChildName()
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as
    //! well
    //! @returns vector of joints where link is a child
    AZStd::vector<const sdf::Joint*> GetJointsForChildLink(
        const sdf::Model& sdfModel, AZStd::string_view linkName, bool gatherNestedModelJoints = false);

    //! Retrieve all joints from URDF/SDF in which the specified link is a parent in a sdf::Joint.
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param linkName Name of link which to query in joint objects ParentName()
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as
    //! well
    //! @returns vector of joints where link is a parent
    AZStd::vector<const sdf::Joint*> GetJointsForParentLink(
        const sdf::Model& sdfModel, AZStd::string_view linkName, bool gatherNestedModelJoints = false);
} // namespace ROS2RobotImporter::Utils
