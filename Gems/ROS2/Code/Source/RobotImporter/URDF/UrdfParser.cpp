/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "UrdfParser.h"

#include <fstream>

#include <AzCore/Debug/Trace.h>
#include <AzCore/std/string/string.h>

#include <urdf_model/model.h>

namespace ROS2
{
    urdf::ModelInterfaceSharedPtr UrdfParser::Parse(const AZStd::string& xmlString)
    {
        return urdf::parseURDF(xmlString.c_str());
    }

    urdf::ModelInterfaceSharedPtr UrdfParser::ParseFromFile(const AZStd::string& filePath)
    {
        std::ifstream istream(filePath.c_str());
        if (!istream)
        {
            AZ_Error("UrdfParser", false, "File %s does not exist", filePath.c_str());
            return nullptr;
        }

        std::string xmlStr((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
        return Parse(xmlStr.c_str());
    }
} // namespace ROS2
