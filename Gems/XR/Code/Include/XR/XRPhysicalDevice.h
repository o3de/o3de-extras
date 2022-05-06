/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRSystemInterface.h>

namespace XR
{
    class PhysicalDeviceDescriptor
    {
    public:
        AZ_RTTI(PhysicalDeviceDescriptor, "{4E11244B-FDED-4CD6-89D7-DC3B4A1C33A8}");

        PhysicalDeviceDescriptor() = default;
        virtual ~PhysicalDeviceDescriptor() = default;

        // Other data related to physical device
    };

    // This class will be responsible for iterating over all the compatible physical
    // devices and picking one that will be used for the app
    class PhysicalDevice
    {
    public:
        AZ_RTTI(PhysicalDevice, "{E7B78CC5-53A9-492E-AA1C-8815FB882E0A}");

        PhysicalDevice() = default;
        virtual ~PhysicalDevice() = default;

        AZStd::intrusive_ptr<PhysicalDeviceDescriptor> m_descriptor;
    };
} // namespace XR
