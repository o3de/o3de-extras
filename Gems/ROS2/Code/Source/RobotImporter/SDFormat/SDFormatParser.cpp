/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SDFormatParser.h"

#include <AzCore/Debug/Trace.h>
#include <AzCore/std/string/string.h>

#include <string.h>

namespace ROS2
{
    AZStd::shared_ptr<sdf::Root> SDFormatParser::Parse(const AZStd::string& xmlString, AZStd::string& parsingLog)
    {
        sdf::SDFPtr sdfElement(new sdf::SDF());
        sdf::init(sdfElement);
        std::string xmlStringStr(xmlString.c_str());
        sdf::Errors readErrors;
        const bool success = sdf::readString(xmlStringStr, sdfElement, readErrors);
        for (const auto& error : readErrors)
        {
            parsingLog += error.Message().c_str();
        }
        if (!success)
        {
            AZ_Error("SDFormatParser", false, "Cannot parse the string %s.", xmlString.c_str());
            return nullptr;
        }

        auto sdfDom = AZStd::make_shared<sdf::Root>();
        sdf::Errors parseErrors = sdfDom->Load(sdfElement);
        for (const auto& error : parseErrors)
        {
            parsingLog += error.Message().c_str();
        }

        return sdfDom;
    }

    AZStd::shared_ptr<sdf::Root> SDFormatParser::Parse(const AZStd::string& xmlString)
    {
        AZStd::string parsingLog = ""; // dummy buffer
        return Parse(xmlString, parsingLog);
    }

    AZStd::shared_ptr<sdf::Root> SDFormatParser::ParseFromFile(const AZStd::string& filePath, AZStd::string& parsingLog)
    {
        std::ifstream istream(filePath.c_str());
        if (!istream)
        {
            AZ_Error("SDFormatParser", false, "File %s does not exist", filePath.c_str());
            return nullptr;
        }

        std::string xmlStr((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
        return Parse(xmlStr.c_str(), parsingLog);
    }

    AZStd::shared_ptr<sdf::Root> SDFormatParser::ParseFromFile(const AZStd::string& filePath)
    {
        AZStd::string parsingLog = ""; // dummy buffer
        return ParseFromFile(filePath, parsingLog);
    }
} // namespace ROS2
