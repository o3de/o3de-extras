/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkUtils.h>
#include <AzCore/Debug/Trace.h>

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

    bool IsError(XrResult result)
    {
        return IsSuccess(result) == false;
    }

    const char* GetResultString(const XrResult result)
    {
        return to_string(result);
    }

    XR::RawStringList FilterList(const XR::RawStringList& source, const XR::StringList& filter)
    {
        XR::RawStringList filteredList;
        for (auto& item : source)
        {
            if (AZStd::find(filter.begin(), filter.end(), item) != filter.end())
            {
                filteredList.push_back(item);
            }
        }
        return filteredList;
    }

    AZStd::vector<const char*> ParseExtensionString(char* names)
    {
        AZStd::vector<const char*> list;
        while (*names != 0)
        {
            list.push_back(names);
            while (*(++names) != 0)
            {
                if (*names == ' ')
                {
                    *names++ = '\0';
                    break;
                }
            }
        }
        return list;
    }

    void FilterAvailableExtensions(GladVulkanContext& context)
    {
        // In some cases (like when running with the GPU profiler on Quest2) the extension is reported as available
        // but the function pointers do not load. Disable the extension if that's the case.
        if (context.EXT_debug_utils && !context.CmdBeginDebugUtilsLabelEXT)
        {
            context.EXT_debug_utils = 0;
        }
    }
}
