/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkDevice.h>

namespace AZ
{
    namespace OpenXRVk
    {
        static AZStd::intrusive_ptr<Device> Device::Create()
        {
        }

        XR::ResultCode Device::InitDeviceInternal(XR::PhysicalDevice& physicalDevice)
        {
            // Create Vulkan Device
            return AZ::RPI::XR::ResultCode::Success;
        }
    } // namespace OpenXRVk
} // namespace AZ
