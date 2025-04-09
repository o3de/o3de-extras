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
namespace SimulationInterfaces
{
    //! Result codes to be used in the Result message
    //!  @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Result.msg">Result.msg</a>
    using ErrorCodeValue = uint8_t;

    //! A message type to represent the result of a failed operation
    struct FailedResult
    {
        FailedResult() = default;
        FailedResult(ErrorCodeValue error_code, const AZStd::string& error_string)
            : error_code(error_code)
            , error_string(error_string)
        {
        }
        ErrorCodeValue error_code;
        AZStd::string error_string;
    };
} // namespace SimulationInterfaces
