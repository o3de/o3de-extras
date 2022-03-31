/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <UrdfParser.h>

#include <fstream>
#include <string>

#include "urdf_model/model.h"

namespace ROS2
{
    urdf::ModelInterfaceSharedPtr UrdfParser::Parse(const std::string & xmlString)
    {
        return urdf::parseURDF(xmlString);
    }

    urdf::ModelInterfaceSharedPtr UrdfParser::ParseFromFile(const std::string & filePath)
    {
        std::ifstream istream(filePath);
        if (!istream)
        {
          AZ_Error("UrdfParser", false, "File %s not exist", filePath.c_str());
          return urdf::ModelInterfaceSharedPtr();
        }

        std::string xmlStr((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());

        return Parse(xmlStr);
    }

} // namespace ROS2