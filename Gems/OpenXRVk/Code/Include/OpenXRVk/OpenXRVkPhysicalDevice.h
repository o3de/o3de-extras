/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRPhysicalDevice.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    class PhysicalDeviceDescriptor final
        : public XR::PhysicalDeviceDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(PhysicalDeviceDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(PhysicalDeviceDescriptor, "{CB485C38-E723-4593-ADCF-DFE220A6A24B}", XR::PhysicalDeviceDescriptor);

        PhysicalDeviceDescriptor() = default;
        virtual ~PhysicalDeviceDescriptor() = default;

        // Other data related to openxr physical device
    };

    // This class will be responsible for iterating over all the compatible physical
    // devices and picking one that will be used for the app
    class PhysicalDevice final
        : public XR::PhysicalDevice
    {
    public:
        AZ_CLASS_ALLOCATOR(PhysicalDevice, AZ::SystemAllocator, 0);
        AZ_RTTI(PhysicalDevice, "{7CE8D7C1-7CC6-4841-9505-DED2761617AC}", XR::PhysicalDevice);

        PhysicalDevice() = default;
        virtual ~PhysicalDevice() = default;

        static AZStd::vector<AZStd::intrusive_ptr<XR::PhysicalDevice>> EnumerateDeviceList();
    };
} // namespace XR

