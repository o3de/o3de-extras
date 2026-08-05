/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SdfParser.h"

#include <sstream>

#include <AzCore/Debug/Trace.h>
#include <AzCore/std/string/regex.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/FixURDF/FixURDF.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include <RobotImporter/Utils/FilePath.h>
#include <RobotImporter/Assets/AssetPathResolver.h>

namespace ROS2RobotImporter::SdfParser
{
    class RedirectSDFOutputStream
    {
    public:
        // Copy the original console stream to restore in the destructor via RAII
        // the console stream itself is referenced here and redirected to a local
        // os stream by default
        // @param consoleStream Reference to sdf::Console::ConsoleStream whose
        //        output will be redirected
        // @param redirectStream reference to stream where console stream output is redirected to
        RedirectSDFOutputStream(sdf::Console::ConsoleStream& consoleStream, std::ostream& redirectStream)
            : m_consoleStreamRef(consoleStream)
            , m_origConsoleStream(consoleStream)
        {
            // Redirect the Output stream to the supplied
            m_consoleStreamRef = sdf::Console::ConsoleStream(&redirectStream);
        }

        ~RedirectSDFOutputStream()
        {
            // Restore the original console stream
            m_consoleStreamRef = AZStd::move(m_origConsoleStream);
        }

    private:
        sdf::Console::ConsoleStream& m_consoleStreamRef;
        sdf::Console::ConsoleStream m_origConsoleStream;
    };

    // Parser result member functions
    sdf::Root& ParseResult::GetRoot() &
    {
        return m_root;
    }
    const sdf::Root& ParseResult::GetRoot() const&
    {
        return m_root;
    }
    sdf::Root&& ParseResult::GetRoot() &&
    {
        return AZStd::move(m_root);
    }

    const AZStd::string& ParseResult::GetParseMessages() const&
    {
        return m_parseMessages;
    }

    AZStd::string&& ParseResult::GetParseMessages() &&
    {
        return AZStd::move(m_parseMessages);
    }

    const sdf::Errors& ParseResult::GetSdfErrors() const&
    {
        return m_sdfErrors;
    }

    sdf::Errors&& ParseResult::GetSdfErrors() &&
    {
        return AZStd::move(m_sdfErrors);
    }

    ParseResult::operator bool() const
    {
        return m_sdfErrors.empty();
    }

    bool ParseResult::UrdfParsedWithModifiedContent() const
    {
        return m_urdfModifications.duplicatedJoints.size() > 0 || m_urdfModifications.missingInertias.size() > 0 ||
            m_urdfModifications.incompleteInertias.size() > 0;
    }

    RootObjectOutcome Parse(AZStd::string_view xmlString, const sdf::ParserConfig& parserConfig)
    {
        return Parse(std::string(xmlString.data(), xmlString.size()), parserConfig);
    }
    RootObjectOutcome Parse(const std::string& xmlString, const sdf::ParserConfig& parserConfig)
    {
        ParseResult parseResult;
        std::ostringstream parseStringStream;
        {
            RedirectSDFOutputStream redirectConsoleStreamMsg(sdf::Console::Instance()->GetMsgStream(), parseStringStream);
            parseResult.m_sdfErrors = parseResult.m_root.LoadSdfString(xmlString, parserConfig);
        }
        // Get any captured sdf::Console messages
        const auto& parseMessages = parseStringStream.str();

        // regular expression to escape console's color codes
        const AZStd::regex escapeColor("\x1B\\[[0-9;]*[A-Za-z]");

        parseResult.m_parseMessages = AZStd::regex_replace(AZStd::string(parseMessages.c_str(), parseMessages.size()), escapeColor, "");

        // if there are no parse errors return the sdf Root object otherwise return the errors
        return parseResult;
    }

    RootObjectOutcome ParseFromFile(
        AZ::IO::PathView filePath, const sdf::ParserConfig& parserConfig, const SdfAssetBuilderSettings& settings)
    {
        // Store path in a AZ::IO::FixedMaxPath which is stack based structure that provides memory
        // for the path string and is null terminated.
        // It is backed by an AZStd::fixed_string<1024> which is a char buffer of 1025 internally
        AZ::IO::FixedMaxPath sourceFilePath = filePath.FixedMaxPathString();
        std::ifstream istream(sourceFilePath.c_str());
        if (!istream)
        {
            auto fileNotFoundMessage = AZStd::fixed_string<1024>::format("File %.*s does not exist", AZ_PATH_ARG(sourceFilePath));
            ParseResult fileNotFoundResult;
            fileNotFoundResult.m_sdfErrors.emplace_back(
                sdf::ErrorCode::FILE_READ,
                std::string{ fileNotFoundMessage.c_str(), fileNotFoundMessage.size() },
                std::string{ sourceFilePath.c_str(), sourceFilePath.Native().size() });
            return fileNotFoundResult;
        }

        std::string xmlStr((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
        if (Utils::IsFileUrdf(filePath) && settings.m_fixURDF)
        {
            // modify in memory
            auto [modifiedXmlStr, modifiedElements] = (Utils::ModifyURDFInMemory(xmlStr));

            auto result = Parse(modifiedXmlStr, parserConfig);
            result.m_urdfModifications = AZStd::move(modifiedElements);
            result.m_modifiedURDFContent = AZStd::move(modifiedXmlStr);
            return result;
        }
        return Parse(xmlStr, parserConfig);
    }


    sdf::ParserConfig CreateSdfParserConfigFromSettings(const SdfAssetBuilderSettings& settings, const AZ::IO::PathView& baseFilePath)
    {
        sdf::ParserConfig sdfConfig;

        sdfConfig.URDFSetPreserveFixedJoint(settings.m_urdfPreserveFixedJoints);

        // Fill in the URI resolution with the supplied prefix mappings.
        for (auto& [prefix, pathList] : settings.m_resolverSettings.m_uriPrefixMap)
        {
            std::string uriPath;
            for (auto& path : pathList)
            {
                if (!uriPath.empty())
                {
                    uriPath.append(std::string(":"));
                }

                uriPath.append(std::string(path.c_str(), path.size()));
            }
            if (!prefix.empty() && !uriPath.empty())
            {
                std::string uriPrefix(prefix.c_str(), prefix.size());
                sdfConfig.AddURIPath(uriPrefix, uriPath);
                AZ_Trace("SdfParserConfig", "Added URI mapping '%s' -> '%s'", uriPrefix.c_str(), uriPath.c_str());
            }
        }

        // If any files couldn't be found using our supplied prefix mappings, this callback will get called.
        // Attempt to use our full path resolution, and print a warning if it still couldn't be resolved.
        sdfConfig.SetFindCallback(
            [settings, baseFilePath](const std::string& fileName) -> std::string
            {
                auto amentPrefixPath = Utils::GetAmentPrefixPath();

                auto resolved = Utils::ResolveAssetPath(AZ::IO::Path(fileName.c_str()), baseFilePath, amentPrefixPath, settings);
                if (!resolved.empty())
                {
                    AZ_Trace("SdfParserConfig", "SDF SetFindCallback resolved '%s' -> '%s'", fileName.c_str(), resolved.c_str());
                    return resolved.c_str();
                }

                AZ_Warning("SdfParserConfig", false, "SDF SetFindCallback failed to resolve '%s'", fileName.c_str());
                return fileName;
            });

        return sdfConfig;
    }

} // namespace ROS2RobotImporter::SdfParser
