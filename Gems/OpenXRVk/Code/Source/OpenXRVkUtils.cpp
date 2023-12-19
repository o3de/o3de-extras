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

    AZ::Quaternion AzQuaternionFromXrPose(const XrPosef& pose, bool convertCoordinates)
    {
        if (convertCoordinates)
        {
            return AZ::Quaternion(pose.orientation.x, -pose.orientation.z, pose.orientation.y, pose.orientation.w);
        }
        return AZ::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }

    AZ::Vector3 AzPositionFromXrPose(const XrPosef& pose, bool convertCoordinates)
    {
        return AZ::Vector3(pose.position.x,
                           convertCoordinates ? -pose.position.z : pose.position.y,
                           convertCoordinates ? pose.position.y : pose.position.z);
    }

    AZ::Transform AzTransformFromXrPose(const XrPosef& pose, bool convertCoordinates)
    {
        const auto azQuat = AzQuaternionFromXrPose(pose, convertCoordinates);
        const auto azVec = AzPositionFromXrPose(pose, convertCoordinates);
        AZ::Transform tm = AZ::Transform::CreateFromQuaternionAndTranslation(azQuat, azVec);
        return tm;
    }

    XrPosef XrPoseFromAzTransform(const AZ::Transform& tm, bool convertCoordinates)
    {
        const auto &azQuat = tm.GetRotation();
        const auto &azPos = tm.GetTranslation();
        
        XrPosef pose;
        
        pose.orientation.x = azQuat.GetX();
        pose.orientation.y = convertCoordinates ? -azQuat.GetZ() : azQuat.GetY();
        pose.orientation.z = convertCoordinates ? azQuat.GetY() : azQuat.GetZ();
        pose.orientation.w = azQuat.GetW();

        pose.position.x = azPos.GetX();
        pose.position.y = convertCoordinates ? -azPos.GetZ() : azPos.GetY();
        pose.position.z = convertCoordinates ?  azPos.GetY() : azPos.GetZ();

        return pose; 
    }

}
