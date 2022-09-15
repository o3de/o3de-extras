/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Android/AndroidEnv.h>

#include <OpenXRVk/OpenXRVkUtils.h>
#include <OpenXRInitializeLoader_Android.h>

namespace OpenXRVk::Platform
{
    bool OpenXRInitializeLoader()
    {
        AZ::Android::AndroidEnv* androidEnv = AZ::Android::AndroidEnv::Get();
        AZ_Assert(androidEnv != nullptr, "Invalid android environment");

        PFN_xrInitializeLoaderKHR initializeLoader = nullptr;
        XrResult result = xrGetInstanceProcAddr(
            XR_NULL_HANDLE, "xrInitializeLoaderKHR",
            reinterpret_cast<PFN_xrVoidFunction*>(&initializeLoader));
        if (IsError(result))
        {
            return false;
        }

        XrLoaderInitInfoAndroidKHR loaderInitInfoAndroid;
        memset(&loaderInitInfoAndroid, 0, sizeof(loaderInitInfoAndroid));
        loaderInitInfoAndroid.type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR;
        loaderInitInfoAndroid.next = nullptr;
        loaderInitInfoAndroid.applicationVM = androidEnv->GetActivityJavaVM();
        loaderInitInfoAndroid.applicationContext = androidEnv->GetActivityRef();

        result = initializeLoader(reinterpret_cast<const XrLoaderInitInfoBaseHeaderKHR*>(&loaderInitInfoAndroid));
        if (IsError(result))
        {
            return false;
        }

        return true;
    }
}
