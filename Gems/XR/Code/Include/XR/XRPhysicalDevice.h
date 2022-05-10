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
    class PhysicalDeviceDescriptor 
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(PhysicalDeviceDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(PhysicalDeviceDescriptor, "{4E11244B-FDED-4CD6-89D7-DC3B4A1C33A8}");

        PhysicalDeviceDescriptor() = default;
        virtual ~PhysicalDeviceDescriptor() = default;

        // Other data related to physical device
    };

    // This class will be responsible for iterating over all the compatible physical
    // devices and picking one that will be used for the app
    class PhysicalDevice
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(PhysicalDevice, AZ::SystemAllocator, 0);
        AZ_RTTI(PhysicalDevice, "{E7B78CC5-53A9-492E-AA1C-8815FB882E0A}");

        PhysicalDevice() = default;
        virtual ~PhysicalDevice() = default;

        AZStd::intrusive_ptr<PhysicalDeviceDescriptor> m_descriptor;
    };
} // namespace XR
