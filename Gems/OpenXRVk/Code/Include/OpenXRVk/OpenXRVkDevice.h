/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRDevice.h>
#include <OpenXRVk_Platform.h>

namespace AZ
{
    namespace OpenXRVk
    {
        // Class that will help manage VkDevice
        class Device final
            : public AZ::RPI::XR::Device
        {
        public:
            static AZStd::intrusive_ptr<Device> Create();
            AZ::RPI::XR::ResultCode InitDeviceInternal(AZ::RPI::XR::PhysicalDevice& physicalDevice) override;

        private:
            VkDevice m_nativeDevice;
        };
    } // namespace OpenXRVk
} // namespace AZ
