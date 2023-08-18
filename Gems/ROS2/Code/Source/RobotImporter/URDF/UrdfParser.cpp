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
#include <console_bridge/console.h>

namespace ROS2::UrdfParser
{
    RootObjectOutcome Parse(AZStd::string_view xmlString, const sdf::ParserConfig& parserConfig)
    {
        return Parse(std::string(xmlString.data(), xmlString.size()), parserConfig);
    }
    RootObjectOutcome Parse(const std::string& xmlString, const sdf::ParserConfig& parserConfig)
    {
        sdf::Root root;
        auto parseRootErrors = root.LoadSdfString(xmlString, parserConfig);

        // if there are no parse errors return the sdf Root object otherwise return the errors
        return parseRootErrors.empty() ? RootObjectOutcome(root) : RootObjectOutcome(AZStd::unexpect, parseRootErrors);
    }

    RootObjectOutcome ParseFromFile(AZ::IO::PathView filePath, const sdf::ParserConfig& parserConfig)
    {
        // Store path in a AZ::IO::FixedMaxPath which is stack based structure that provides memory
        // for the path string and is null terminated.
        // It is backed by an AZStd::fixed_string<1024> which is a char buffer of 1025 internally
        AZ::IO::FixedMaxPath urdfFilePath = filePath.FixedMaxPathString();
        std::ifstream istream(urdfFilePath.c_str());
        if (!istream)
        {
            auto fileNotFoundMessage = AZStd::fixed_string<1024>::format("File %.*s does not exist", AZ_PATH_ARG(urdfFilePath));
            RootObjectOutcome fileNotExistOutcome;
            fileNotExistOutcome = AZStd::unexpected<sdf::Errors>(AZStd::in_place,
                {
                    sdf::Error{ sdf::ErrorCode::FILE_READ,
                        std::string{ fileNotFoundMessage.c_str(), fileNotFoundMessage.size() },
                        std::string{ urdfFilePath.c_str(), urdfFilePath.Native().size() } }
                });
            return fileNotExistOutcome;
        }

        std::string xmlStr((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
        return Parse(xmlStr, parserConfig);
    }
} // namespace ROS2
