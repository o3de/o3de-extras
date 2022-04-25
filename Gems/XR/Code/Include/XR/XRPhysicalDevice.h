/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RHI/PhysicalDevice.h>
#include <Atom/RHI.Reflect/PhysicalDeviceDescriptor.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            class PhysicalDeviceDescriptor
                : public AZ::RHI::PhysicalDeviceDescriptor
            {
            public:
                AZ_RTTI(PhysicalDeviceDescriptor, "{4E11244B-FDED-4CD6-89D7-DC3B4A1C33A8}", AZ::RHI::PhysicalDeviceDescriptor);

                PhysicalDeviceDescriptor() = default;
                virtual ~PhysicalDeviceDescriptor() = default;

                // Other data related to xr device
            };

            // This class will be responsible for iterating over all the compatible physical
            // devices and picking one that will be used for the app
            class PhysicalDevice
                : public AZ::RHI::PhysicalDevice
            {
            public:
                AZ_RTTI(PhysicalDevice, "{E7B78CC5-53A9-492E-AA1C-8815FB882E0A}", AZ::RHI::PhysicalDevice);

                PhysicalDevice() = default;
                virtual ~PhysicalDevice() = default;

                PhysicalDeviceDescriptor m_descriptor;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
