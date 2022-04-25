/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            enum class ResultCode : int
            {
                // The operation succeeded.
                Success = 0,

                // The operation failed with an unknown error.
                Fail,

                // The operation failed because the feature is unimplemented on the particular platform.
                Unimplemented,

                // The operation failed due to invalid arguments.
                InvalidArgument
            };
        }
    } // namespace RPI
} // namespace AZ
