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
    namespace Internal
    {
        void checkIfCurrentLocaleHasDotAsADecimalSeparator()
        {
            // Due to the fact that URDF parser takes into account the locale information, incompatibility between URDF file locale and
            // system locale might lead to incorrect URDF parsing. Mainly it affects floating point numbers, and the decimal separator. When
            // locales are set to system with comma as decimal separator and URDF file is created with dot as decimal separator, URDF parser
            // will trim the floating point number after comma. For example, if parsing 0.1, URDF parser will parse it as 0.
            // This might lead to incorrect URDF loading. If the current locale is not a dot (as per standard ROS locale), we warn the user.
            std::locale currentLocale("");
            if (std::use_facet<std::numpunct<char>>(currentLocale).decimal_point() != '.')
            {
                AZ_Warning(
                    "UrdfParser", false, "Locale %s might be incompatible with the URDF file content.\n", currentLocale.name().c_str());
            }
        }
    } // namespace Internal

    urdf::ModelInterfaceSharedPtr UrdfParser::Parse(const AZStd::string& xmlString)
    {
        m_customConsoleHandler.console_ss = std::stringstream();
        console_bridge::useOutputHandler(&m_customConsoleHandler);
        Internal::checkIfCurrentLocaleHasDotAsADecimalSeparator();
        const auto ret = urdf::parseURDF(xmlString.c_str());
        console_bridge::restorePreviousOutputHandler();
        return ret;
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

    AZStd::string UrdfParser::GetUrdfParsingLog()
    {
        return AZStd::string(
            UrdfParser::m_customConsoleHandler.console_ss.str().c_str(), UrdfParser::m_customConsoleHandler.console_ss.str().size());
    }

    void UrdfParser::CustomConsoleHandler::log(const std::string& text, console_bridge::LogLevel level, const char* filename, int line)
    {
        AZ_Printf("UrdfParser", "%s", text.c_str());
        console_ss << text << "\n";
    }

    UrdfParser::CustomConsoleHandler UrdfParser::m_customConsoleHandler;
} // namespace ROS2
