/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkFactory.h>
#include <AzCore/Memory/Memory.h>

namespace AZ
{
    namespace OpenXRVk
    {
        Factory::Factory()
        {
            RPI::XR::Factory::Register(this);
        }

        Factory::~Factory()
        {
            RPI::XR::Factory::Unregister(this);
        }

        // Create XR::Instance object
        AZStd::intrusive_ptr<AZ::RPI::XR::Instance> CreateInstance()
        {
            return aznew AZ::OpenXRVk::Instance;
        }

        // Create XR::Device object
        AZStd::intrusive_ptr<AZ::RPI::XR::Device> CreateDevice()
        {
            return aznew AZ::OpenXRVk::Device;
        }

        // Return a list of XR::PhysicalDevice
        AZStd::vector<AZStd::intrusive_ptr<AZ::RPI::XR::PhysicalDevice>> EnumerateDeviceList()
        {
            return AZ::OpenXRVk::PhysicalDevice::EnumerateList();
        }

        // Create XR::Session object
        AZStd::intrusive_ptr<AZ::RPI::XR::Session> CreateSession()
        {
            return aznew AZ::OpenXRVk::Session;
        }

        // Create XR::Input object
        AZStd::intrusive_ptr<AZ::RPI::XR::Input> CreateInput()
        {
            return aznew AZ::OpenXRVk::Input;
        }

        // Create XR::SwapChain object
        AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain> CreateSwapchain()
        {
            return aznew AZ::OpenXRVk::SwapChain;
        }

        // Create XR::ViewSwapChain object
        AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain::View> CreateViewSwapchain()
        {
            return aznew AZ::OpenXRVk::SwapChain::View;
        }

        // Create RPI::XR::GraphicsBindingDescriptor that will contain
        // renderer information needed to start a session
        AZStd::intrusive_ptr<AZ::RPI::XR::GraphicsBinding::Descriptor> CreateGraphicsBindingDescriptor()
        {
            return aznew AZ::OpenXRVk::SwapChain::View;
        }
    } // namespace OpenXRVk
} // namespace AZ
