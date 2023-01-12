/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "RobotImporter/URDF/UrdfParser.h"
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>

namespace ROS2::Utils::xacro
{

    static const char* kXacroExecutable = "xacro";
    using Params = AZStd::unordered_map<AZStd::string, AZStd::string>;
    //! Structure that keeps all artifacts of xacro execution
    struct ExecutionOutcome
    {
        //! parsed URDF from successful xacro's output
        urdf::ModelInterfaceSharedPtr m_urdfHandle;
        //! return code of 'xacro' program
        int m_returnCode{ 255 };
        //! called program name
        AZStd::string m_called;
        //! standard output of xacro execution
        AZStd::string m_logStandardOutput;
        //! standard error output of xacro process
        AZStd::string m_logErrorOutput;

        //! Gets if execution was a success
        operator bool()
        {
            return m_returnCode == 0 && m_urdfHandle != nullptr;
        }
    };

    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroData(AZStd::vector<char> data);
    AZStd::unordered_map<AZStd::string, AZStd::string> GetParameterFromXacroFile(AZStd::string filename);

    ExecutionOutcome ParseXacro(const AZStd::string& filename, const Params& params);

    //! Finds absolute path of xacro executable.
    //! It encapsulates `which`
    //! @returns empty string is returned if no executable is found.
    AZStd::string WhichXacro();

} // namespace ROS2::Utils::xacro
