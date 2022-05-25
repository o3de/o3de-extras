/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRDevice.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    //! Vulkan specific XR device back-end class that will help manage 
    //! xr specific vulkan native objects related to device.
    class Device final
    : public XR::Device
    {
    public:
        AZ_CLASS_ALLOCATOR(Device, AZ::SystemAllocator, 0);
        AZ_RTTI(Device, "{81FD9B99-EDA5-4381-90EC-335073554379}", XR::Device);

        static XR::Ptr<Device> Create();

        //////////////////////////////////////////////////////////////////////////
        // XR::Device overrides
        // Create the xr specific native device object and populate the XRDeviceDescriptor with it.
        AZ::RHI::ResultCode InitDeviceInternal(AZ::RHI::XRDeviceDescriptor* instanceDescriptor) override;
        //////////////////////////////////////////////////////////////////////////

    private:

        //! Clean native objects.
        void ShutdownInternal() override;

        VkDevice m_xrVkDevice = VK_NULL_HANDLE;
    };
}