/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

namespace AZ
{
    namespace OpenXRVk
    {
        bool IsSuccess(XrResult result)
        {
            if (result != XR_SUCCESS)
            {
                AZ_Error("XR", false, "ERROR: XR API method failed: %s", GetResultString(result));
                return false;
            }
            return true;
        }
    } // namespace OpenXRVk
} // namespace AZ
