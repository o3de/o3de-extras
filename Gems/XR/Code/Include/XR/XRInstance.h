/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRResult.h>
#include <Atom/RPI.Public/XR/XRPhysicalDevice.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            // XR::Instance class. It will be responsible for collecting all the data like
            // form factor, physical device etc that will be needed to initialize an instance
            class Instance
            {
            public:
                Instance() = default;
                virtual ~Instance() = default;

                class Descriptor
                {
                public:
                    // Form Factor enum
                    PhysicalDevice* physicalDevice;
                };

                virtual ResultCode InitInstanceInternal() = 0;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
