/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>
#include <simulation_interfaces/msg/result.hpp>
namespace SimulationInterfaces
{
    //! Result codes to be used in the Result message
    //!  @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg">Result.msg</a>
    using ErrorCodeType = simulation_interfaces::msg::Result::_result_type;

    //! A message type to represent the result of a failed operation
    struct FailedResult
    {
        FailedResult() = default;
        FailedResult(ErrorCodeType errorCode, const AZStd::string& errorString)
            : m_errorCode(errorCode)
            , m_errorString(errorString)
        {
        }
        ErrorCodeType m_errorCode;
        AZStd::string m_errorString;
    };
} // namespace SimulationInterfaces
