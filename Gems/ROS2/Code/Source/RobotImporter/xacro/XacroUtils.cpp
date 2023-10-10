/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "XacroUtils.h"
#include <AzCore/IO/FileIO.h>
#include <AzCore/Settings/SettingsRegistryMergeUtils.h>
#include <AzCore/XML/rapidxml.h>
#include <AzFramework/Process/ProcessCommunicator.h>
#include <AzFramework/Process/ProcessWatcher.h>
#include <FixURDF/FixURDF.h>
#include <QString>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

namespace ROS2::Utils::xacro
{
    ExecutionOutcome ParseXacro(
        const AZStd::string& filename, const Params& params, const sdf::ParserConfig& parserConfig, const SdfAssetBuilderSettings& settings)
    {
        ExecutionOutcome outcome;
        // test if xacro exists
        AZ::IO::Path xacroPath = "xacro";
        auto settingsRegistry = AZ::SettingsRegistry::Get();
        AZStd::string xacroExecutablePath;
        if (settingsRegistry && settingsRegistry->Get(xacroExecutablePath, "/O3DE/ROS2/xacro_executable_path"))
        {
            // Validate the path using AZ::IO::SystemFile:Exists
            const bool pathExists = AZ::IO::SystemFile::Exists(xacroExecutablePath.c_str());
            // Use the AzProcessWatcher to launch the executable.
            if (pathExists)
            {
                xacroPath = xacroExecutablePath;
            }
        }

        AzFramework::ProcessLauncher::ProcessLaunchInfo processLaunchInfo;
        processLaunchInfo.m_processExecutableString = xacroPath.Native();
        processLaunchInfo.m_commandlineParameters.emplace<AZStd::vector<AZStd::string>>({ AZStd::string("--help") });
        if (AZStd::unique_ptr<AzFramework::ProcessWatcher> xacroHelpProcess(AzFramework::ProcessWatcher::LaunchProcess(processLaunchInfo,
            AzFramework::ProcessCommunicationType::COMMUNICATOR_TYPE_STDINOUT));
            xacroHelpProcess != nullptr)
        {
            // 60 seconds is used as a cap on how long it should take to run the `xacro --help` command
            // In reality this should take less than a second, but there has been a couple of occurrences
            // where it takes over a second to run. 60 seconds is a fail fase value
            constexpr AZStd::chrono::seconds helpCommandWaitTime{ 60 };

            AZ::u32 exitCode{ AZStd::numeric_limits<AZ::u32>::max() };
            xacroHelpProcess->WaitForProcessToExit(static_cast<AZ::u32>(helpCommandWaitTime.count()), &exitCode);
            if (exitCode != 0)
            {
                const auto fullProcessCommand = AZStd::string::format("%s %s",
                    processLaunchInfo.m_processExecutableString.c_str(),
                    processLaunchInfo.GetCommandLineParametersAsString().c_str());
                AZ_Warning(
                    "ParseXacro",
                    false,
                    "Attempted to run validate xacro existence with command: %s",
                    fullProcessCommand.c_str());
                outcome.m_called = fullProcessCommand;
                return outcome;
            }
        }

        AZ_Printf("ParseXacro", "xacro executable : %s \n", xacroPath.c_str());
        AZ_Printf("ParseXacro", "Convert xacro file : %s \n", filename.c_str());
        // Clear out the processLanunchInfo structure
        processLaunchInfo = {};
        processLaunchInfo.m_processExecutableString = xacroPath.Native();
        processLaunchInfo.m_commandlineParameters.emplace<AZStd::vector<AZStd::string>>({ filename });
        for (const auto& param : params)
        {
            const AZStd::string& name{ param.first };
            const AZStd::string& value{ param.second };
            AZStd::string command_line_parameter{ name + ":=" + value };
            AZStd::get<AZStd::vector<AZStd::string>>(processLaunchInfo.m_commandlineParameters).emplace_back(command_line_parameter);
        }

        outcome.m_called = processLaunchInfo.m_processExecutableString + " " + processLaunchInfo.GetCommandLineParametersAsString();
        AZ_Printf("ParseXacro", "calling file : %s \n", outcome.m_called.c_str());

        AzFramework::ProcessOutput process_output;
        const bool succeed = AzFramework::ProcessWatcher::LaunchProcessAndRetrieveOutput(
            processLaunchInfo, AzFramework::ProcessCommunicationType::COMMUNICATOR_TYPE_STDINOUT, process_output);

        if (succeed && process_output.HasOutput() && !process_output.HasError())
        {
            AZ_Printf("ParseXacro", "xacro finished with success \n");
            const auto& output = process_output.outputResult;

            if (settings.m_fixURDF)
            {
                // modify in memory URDF result
                auto [modifiedXmlStr, modifiedElements] = (ROS2::Utils::ModifyURDFInMemory(output));
                outcome.m_urdfHandle = UrdfParser::Parse(modifiedXmlStr, parserConfig);
                outcome.m_urdfHandle.m_modifiedURDFContent = AZStd::move(modifiedXmlStr);
                outcome.m_urdfHandle.m_urdfModifications = AZStd::move(modifiedElements);
                outcome.m_succeed = true;
            }
            else
            {
                outcome.m_urdfHandle = UrdfParser::Parse(output, parserConfig);
                outcome.m_succeed = true;
            }
        }
        else
        {
            AZ_Printf("ParseXacro", "xacro finished with error \n");
            const auto& stdStream = process_output.outputResult;
            const auto& cerrStream = process_output.errorResult;
            outcome.m_logStandardOutput = AZStd::string(stdStream.data(), stdStream.size());
            outcome.m_logErrorOutput = AZStd::string(cerrStream.data(), cerrStream.size());
            outcome.m_succeed = false;
        }
        return outcome;
    }

    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroData(const AZStd::string& data)
    {
        AZStd::vector<char> dataArray;
        dataArray.resize_no_construct(data.size() + 1);
        AZStd::copy(data.data(), data.end(), dataArray.begin());
        dataArray.back() = '\0';

        constexpr char argNameTag[]{ "xacro:arg" };
        constexpr char nameTag[]{ "name" };
        constexpr char defaultTag[]{ "default" };

        AZStd::unordered_map<AZStd::string, AZStd::string> params;
        AZ::rapidxml::xml_document<char> doc;
        doc.parse<AZ::rapidxml::parse_full>(dataArray.data());
        if (doc.first_node() == nullptr)
        {
            return params;
        }
        AZ::rapidxml::xml_node<char>* xmlRootNode = doc.first_node("robot");
        if (xmlRootNode == nullptr)
        {
            return params;
        }
        for (AZ::rapidxml::xml_node<>* child_node = xmlRootNode->first_node(); child_node; child_node = child_node->next_sibling())
        {
            if (strcmp(argNameTag, child_node->name()) == 0)
            {
                const auto* attributeName = child_node->first_attribute(nameTag);
                const auto* attributeDefault = child_node->first_attribute(defaultTag);

                if (attributeName)
                {
                    AZStd::string name{ attributeName->value() };
                    AZStd::string value;
                    if (attributeDefault)
                    {
                        value = AZStd::string{ attributeDefault->value() };
                    }
                    params.emplace(AZStd::make_pair(AZStd::move(name), AZStd::move(value)));
                }
            }
        }
        return params;
    }

    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroFile(const AZStd::string& filename)
    {
        AZ::IO::FileIOStream fileStream;
        if (!fileStream.Open(filename.c_str(), AZ::IO::OpenMode::ModeRead | AZ::IO::OpenMode::ModeBinary))
        {
            return xacro::Params();
        }

        AZ::IO::SizeType length = fileStream.GetLength();
        if (length == 0)
        {
            return xacro::Params();
        }

        AZStd::string charBuffer;
        auto ReadFile = [&fileStream](char* buffer, size_t size)
        {
            return fileStream.Read(size, buffer);
        };
        charBuffer.resize_and_overwrite(length, ReadFile);
        return GetParameterFromXacroData(charBuffer);
    }

} // namespace ROS2::Utils::xacro
