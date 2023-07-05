/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>
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
        //! Parse string with URDF data and generate model.
        //! @param xmlString a string that contains URDF data (XML format).
        //! @return model represented as a tree of parsed links.
        sdf::Root* Parse(const AZStd::string& xmlString);

        //! Parse file with URDF data and generate model.
        //! @param filePath is a path to file with URDF data that will be loaded and parsed.
        //! @return model represented as a tree of parsed links.
        sdf::Root* ParseFromFile(const AZStd::string& filePath);

        //! Retrieve console log from URDF parsing
        //! @return a log with output from urdf_parser
        AZStd::string GetUrdfParsingLog();

    }; // namespace UrdfParser
} // namespace ROS2