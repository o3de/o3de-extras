/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkDevice.h>

namespace OpenXRVk
{
    AZStd::intrusive_ptr<Device> Device::Create()
    {
        return aznew Device;
    }

    AZ::RHI::ResultCode Device::InitDeviceInternal()
    {
        // Create Vulkan Device
        //m_nativeDevice = Create();
        return AZ::RHI::ResultCode::Success;
    }
}
