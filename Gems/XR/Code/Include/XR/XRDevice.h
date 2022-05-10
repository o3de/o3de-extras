/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <Atom/RPI.Public/XR/XRSystemInterface.h>

namespace XR
{
    class DeviceDescriptor
        : public AZ::RPI::XRDeviceDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(DeviceDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(DeviceDescriptor, "{1FF2D68D-DA6A-47B3-A5BE-18E3A100C830}", AZ::RPI::XRDeviceDescriptor);

        DeviceDescriptor() = default;
        virtual ~DeviceDescriptor() = default;

        //any extra info for a generic xr device
    };

    class Device
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(Device, AZ::SystemAllocator, 0);
        AZ_RTTI(Device, "{A31B0DC2-BD54-443E-9350-EB1B10670FF9}");

        Device() = default;
        virtual ~Device() = default;

        AZStd::intrusive_ptr<DeviceDescriptor> m_descriptor;

        virtual AZ::RHI::ResultCode InitDeviceInternal() = 0;
    };
} // namespace XR

