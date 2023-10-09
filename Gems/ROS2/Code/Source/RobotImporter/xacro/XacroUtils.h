/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <RobotImporter/URDF/UrdfParser.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>

namespace ROS2::Utils::xacro
{

    static const char* XacroExecutable = "xacro";
    using Params = AZStd::unordered_map<AZStd::string, AZStd::string>;
    //! Structure that keeps all artifacts of xacro execution
    struct ExecutionOutcome
    {
        //! Parsed URDF from successful xacro's output
        UrdfParser::RootObjectOutcome m_urdfHandle;
        //! Return code of 'xacro' program
        bool m_succeed{ false };
        //! Called program name
        AZStd::string m_called;
        //! Standard output of xacro execution
        AZStd::string m_logStandardOutput;
        //! Standard error output of xacro process
        AZStd::string m_logErrorOutput;

        //! Gets if execution was a success
        explicit operator bool() const
        {
            return m_succeed && m_urdfHandle;
        }
    };

    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroData(const AZStd::string& data);
    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroFile(const AZStd::string& filename);

    ExecutionOutcome ParseXacro(
        const AZStd::string& filename,
        const Params& params,
        const sdf::ParserConfig& parserConfig,
        const SdfAssetBuilderSettings& settings);

} // namespace ROS2::Utils::xacro
