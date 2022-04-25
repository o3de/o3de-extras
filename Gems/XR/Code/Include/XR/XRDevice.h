/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RHI/Device_Platform.h>
#include <Atom/RPI.Public/XR/XRPhysicalDevice.h>
#include <Atom/RPI.Public/XR/XRResult.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            // This class will be responsible for creating XR::Device instance
            // which will then be passed to the renderer to be used as needed.
            class Device
            {
            public:
                Device() = default;
                virtual ~Device() = default;

                class Descriptor
                {
                public:
                    Descriptor() = default;
                    ~Descriptor() = default;

                    PhysicalDevice* physicalDevice;
                };

                virtual ResultCode InitDeviceInternal(Device::Descriptor descriptor) = 0;
                Device::Descriptor m_descriptor;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
