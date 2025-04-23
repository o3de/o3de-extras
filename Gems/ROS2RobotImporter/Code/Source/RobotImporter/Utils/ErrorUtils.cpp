/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ErrorUtils.h"

#include <sdf/sdf.hh>

namespace ROS2::Utils
{
    AZStd::string JoinSdfErrorsToString(AZStd::span<const sdf::Error> sdfErrors)
    {
        AZStd::string aggregateErrorMessages;
        AppendSdfErrorsToString(aggregateErrorMessages, sdfErrors);
        return aggregateErrorMessages;
    }

    void AppendSdfErrorsToString(AZStd::string& outputResult, AZStd::span<const sdf::Error> sdfErrors)
    {
        for (const sdf::Error& sdfError : sdfErrors)
        {
            AZStd::string errorMessage = AZStd::string::format("ErrorCode=%d", static_cast<int32_t>(sdfError.Code()));
            errorMessage += AZStd::string::format(", Message=%s", sdfError.Message().c_str());
            if (sdfError.LineNumber().has_value())
            {
                errorMessage += AZStd::string::format(", Line=%d", sdfError.LineNumber().value());
            }
            outputResult += errorMessage;
            outputResult += '\n';
        }
    }
} // namespace ROS2::Utils
