/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkUtils.h>

namespace OpenXRVk
{
    AZ::RHI::ResultCode ConvertResult(XrResult xrResult)
    {
        switch (xrResult)
        {
        case XR_SUCCESS:
            return AZ::RHI::ResultCode::Success;
        case XR_ERROR_OUT_OF_MEMORY:
            return AZ::RHI::ResultCode::OutOfMemory;
        default:
            return AZ::RHI::ResultCode::Fail;
        }
    }

    bool IsSuccess(XrResult result)
    {
        if (result != XR_SUCCESS)
        {
            AZ_Error("XR", false, "ERROR: XR API method failed: %s", GetResultString(result));
            return false;
        }
        return true;
    }
}
