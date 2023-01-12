/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "XacroUtils.h"
#include <AzCore/IO/FileIO.h>
#include <AzCore/XML/rapidxml.h>
#include <QProcess>
#include <QString>

namespace ROS2::Utils::xacro
{

    ExecutionOutcome ParseXacro(const AZStd::string& filename, const Params& params)
    {
        ExecutionOutcome outcome;
        // test if xacro exists
        AZStd::string xacroPath = WhichXacro();

        AZ_Warning("ParseXacro", !xacroPath.empty(), "There is xacro executable in your path");
        if (xacroPath.empty())
        {
            return ExecutionOutcome{};
        }
        AZ_Printf("ParseXacro", "xacro executable : %s", xacroPath.c_str());
        AZ_Printf("ParseXacro", "Convert xacro file : %s", filename.c_str());
        const QString program = QString::fromUtf8(xacroPath.data(), xacroPath.size());
        QStringList arguments;
        arguments << QString::fromUtf8(filename.data(), int(filename.size()));

        for (const auto& p : params)
        {
            QString name = QString::fromUtf8(p.first.data());
            QString value = QString::fromUtf8(p.second.data());
            arguments << name + ":=" + value;
        }

        QProcess myProcess;

        QString called = program;

        for (const auto& p : arguments)
        {
            called += " " + p;
        }

        outcome.m_called = AZStd::string(called.toUtf8().constData());
        AZ_Printf("ParseXacro", "calling file : %s", outcome.m_called.c_str());

        myProcess.start(program, arguments);

        if (myProcess.waitForFinished() && myProcess.exitStatus() == QProcess::NormalExit && myProcess.exitCode() == 0)
        {
            AZ_Printf("ParseXacro", "xacro finished with success");
            auto output = myProcess.readAllStandardOutput();
            outcome.m_urdfHandle = UrdfParser::Parse(output.data());
        }
        else
        {
            AZ_Printf("ParseXacro", "xacro finished with code %d", myProcess.exitCode());
            auto std = myProcess.readAllStandardOutput();
            auto cerr = myProcess.readAllStandardError();
            outcome.m_logStandardOutput = AZStd::string(std.data(), std.size());
            outcome.m_logErrorOutput = AZStd::string(cerr.data(), cerr.size());
        }
        outcome.m_returnCode = myProcess.exitCode();
        return outcome;
    }

    AZStd::string WhichXacro()
    {
        const QString program{ "which" };
        QStringList arguments;
        arguments << kXacroExecutable;
        QProcess myProcess;
        myProcess.start(program, arguments);
        if (myProcess.waitForFinished() && myProcess.exitStatus() == QProcess::NormalExit && myProcess.exitCode() == 0)
        {
            auto output = myProcess.readAllStandardOutput().trimmed();
            return AZStd::string{ output.data(), static_cast<AZStd::string::size_type>(output.size()) };
        }
        return AZStd::string{};
    }

    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroData(AZStd::vector<char> data)
    {
        const char* kArgName = "xacro:arg";
        const char* kName = "name";
        const char* kDefault = "default";

        AZStd::unordered_map<AZStd::string, AZStd::string> params;
        AZ::rapidxml::xml_document<char> doc;
        doc.parse<AZ::rapidxml::parse_full>(data.data());
        AZ::rapidxml::xml_node<char>* xmlRootNode = doc.first_node("robot");
        for (AZ::rapidxml::xml_node<>* child_node = xmlRootNode->first_node(); child_node; child_node = child_node->next_sibling())
        {
            if (strcmp(kArgName, child_node->name()) == 0)
            {
                const auto* attributeName = child_node->first_attribute(kName);
                const auto* attributeDefault = child_node->first_attribute(kDefault);

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

    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroFile(AZStd::string filename)
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

        AZStd::vector<char> charBuffer;
        charBuffer.resize_no_construct(length + 1);
        fileStream.Read(length, charBuffer.data());
        charBuffer.back() = 0;
        return GetParameterFromXacroData(charBuffer);
    }

} // namespace ROS2::Utils::xacro
