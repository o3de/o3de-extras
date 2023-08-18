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
#include <QString>

#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

namespace ROS2::Utils::xacro
{

    ExecutionOutcome ParseXacro(const AZStd::string& filename, const Params& params)
    {
        ExecutionOutcome outcome;
        // test if xacro exists
        AZStd::string xacroPath = "xacro";
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

        AZ_Warning("ParseXacro", !xacroPath.empty(), "There is xacro executable in your path");
        if (xacroPath.empty())
        {
            return ExecutionOutcome{};
        }
        AZ_Printf("ParseXacro", "xacro executable : %s \n", xacroPath.c_str());
        AZ_Printf("ParseXacro", "Convert xacro file : %s \n", filename.c_str());
        const QString program = QString::fromUtf8(xacroPath.data(), xacroPath.size());
        AzFramework::ProcessLauncher::ProcessLaunchInfo processLaunchInfo;
        processLaunchInfo.m_processExecutableString = xacroPath;
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
            // Read the SDF Settings from the Settings Registry into a local struct
            SdfAssetBuilderSettings sdfBuilderSettings;
            sdfBuilderSettings.LoadSettings();
            // Set the parser config settings for URDF content
            sdf::ParserConfig parserConfig;
            parserConfig.URDFSetPreserveFixedJoint(sdfBuilderSettings.m_urdfPreserveFixedJoints);
            outcome.m_urdfHandle = UrdfParser::Parse(output, parserConfig);
            outcome.m_succeed = true;
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
