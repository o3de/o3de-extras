/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/utility/expected.h>
#include <sdf/Root.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Material.hh>
#include <sdf/Model.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Visual.hh>
#include <sdf/Geometry.hh>
#include <sdf/Collision.hh>

namespace ROS2
{
    //! Class for parsing URDF data.
    namespace UrdfParser
    {
        //! Stores the result of parsing URDF data
        //! On success the sdf::Root object is returned
        //! On failure the sdf::Errors vector is returned
        using RootObjectOutcome = AZStd::expected<sdf::Root, sdf::Errors>;

        //! Parse string with URDF data and generate model.
        //! @param xmlString a string that contains URDF data (XML format).
        //! @return model represented as a tree of parsed links.
        RootObjectOutcome Parse(const std::string& xmlString);

        //! Parse file with URDF data and generate model.
        //! @param filePath is a path to file with URDF data that will be loaded and parsed.
        //! @return model represented as a tree of parsed links.
        RootObjectOutcome ParseFromFile(AZ::IO::PathView filePath);

        //! Retrieve console log from URDF parsing
        //! @return a log with output from urdf_parser
        AZStd::string GetUrdfParsingLog();

    }; // namespace UrdfParser
} // namespace ROS2
