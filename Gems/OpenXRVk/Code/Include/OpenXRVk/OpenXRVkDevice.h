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
    class DeviceDescriptor final
        : public XR::DeviceDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(DeviceDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(DeviceDescriptor, "{B0DB4670-A233-4F3F-A5C7-5D2B76F6D911}", XR::DeviceDescriptor);

        DeviceDescriptor() = default;
        virtual ~DeviceDescriptor() = default;

        //any extra info for a generic xr device
    };

    // Class that will help manage VkDevice
    class Device final
        : public XR::Device
    {
    public:
        AZ_CLASS_ALLOCATOR(Device, AZ::SystemAllocator, 0);
        AZ_RTTI(Device, "{81FD9B99-EDA5-4381-90EC-335073554379}", XR::Device);

        static AZStd::intrusive_ptr<Device> Create();
        AZ::RHI::ResultCode InitDeviceInternal() override;

    private:
        VkDevice m_nativeDevice;
    };
}