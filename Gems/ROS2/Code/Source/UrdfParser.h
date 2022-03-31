/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <string>

#include <urdf_parser/urdf_parser.h>

namespace ROS2
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //! Class for parsing URDF data
    class UrdfParser
    {
        public:
            //! Parse string with URDF data and generate model
            static urdf::ModelInterfaceSharedPtr Parse(const std::string & xmlString);

            //! Parse file with URDF data and generate model
            static urdf::ModelInterfaceSharedPtr ParseFromFile(const std::string & filePath);
    };

} // namespace ROS2