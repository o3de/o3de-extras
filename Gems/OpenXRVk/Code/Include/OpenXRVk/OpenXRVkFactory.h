/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRFactory.h>
#include <OpenXRVk/OpenXRVkDevice.h>
#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkPhysicalDevice.h>
#include <OpenXRVk/OpenXRVkSession.h>
#include <OpenXRVk/OpenXRVkInput.h>
#include <OpenXRVk/OpenXRVkSwapChain.h>
#include <OpenXRVk/OpenXRVkGraphicsBinding.h>

namespace AZ
{
    namespace OpenXRVk
    {
        //! Interface responsible for creating all the XR objects which are
        //! internally backed by concrete objects
        class Factory final
            : public AZ::RPI::XR::Factory
        {
        public:
            AZ_CLASS_ALLOCATOR(Factory, AZ::SystemAllocator, 0);

            AZ_DISABLE_COPY_MOVE(Factory);

            Factory();
            ~Factory();

            // Create XR::Instance object
            AZStd::intrusive_ptr<AZ::RPI::XR::Instance> CreateInstance() override;

            // Create XR::Device object
            AZStd::intrusive_ptr<AZ::RPI::XR::Device> CreateDevice() override;

            // Return a list of XR::PhysicalDevice
            AZStd::vector<AZStd::intrusive_ptr<AZ::RPI::XR::PhysicalDevice>> EnumerateDeviceList() override;

            // Create XR::Session object
            AZStd::intrusive_ptr<AZ::RPI::XR::Session> CreateSession() override;

            // Create XR::Input object
            AZStd::intrusive_ptr<AZ::RPI::XR::Input> CreateInput() override;

            // Create XR::SwapChain object
            AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain> CreateSwapchain() override;

            // Create XR::ViewSwapChain object
            AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain::View> CreateViewSwapchain() override;

            // Create RPI::XR::GraphicsBindingDescriptor that will contain
            // renderer information needed to start a session
            AZStd::intrusive_ptr<AZ::RPI::XR::GraphicsBinding::Descriptor> CreateGraphicsBindingDescriptor() override;

        };
    } // namespace OpenXRVk
} // namespace AZ
