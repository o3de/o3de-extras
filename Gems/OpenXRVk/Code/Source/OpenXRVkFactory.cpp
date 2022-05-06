/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkFactory.h>
#include <AzCore/Memory/Memory.h>

namespace OpenXRVk
{
    Factory::Factory()
    {
        XR::Factory::Register(this);
    }

    Factory::~Factory()
    {
        XR::Factory::Unregister(this);
    }

    // Create XR::Instance object
    AZStd::intrusive_ptr<XR::Instance> CreateInstance()
    {
        return aznew OpenXRVk::Instance;
    }

    // Create XR::Device object
    AZStd::intrusive_ptr<XR::Device> CreateDevice()
    {
        return aznew OpenXRVk::Device;
    }

    // Return a list of XR::PhysicalDevice
    AZStd::vector<AZStd::intrusive_ptr<XR::PhysicalDevice>> EnumerateDeviceList()
    {
        return OpenXRVk::PhysicalDevice::EnumerateList();
    }

    // Create XR::Session object
    AZStd::intrusive_ptr<XR::Session> CreateSession()
    {
        return aznew OpenXRVk::Session;
    }

    // Create XR::Input object
    AZStd::intrusive_ptr<XR::Input> CreateInput()
    {
        return aznew OpenXRVk::Input;
    }

    // Create XR::SwapChain object
    AZStd::intrusive_ptr<XR::SwapChain> CreateSwapchain()
    {
        return aznew OpenXRVk::SwapChain;
    }

    // Create XR::ViewSwapChain object
    AZStd::intrusive_ptr<XR::SwapChain::View> CreateViewSwapchain()
    {
        return aznew OpenXRVk::SwapChain::View;
    }

    // Create RPI::XR::GraphicsBindingDescriptor that will contain
    // renderer information needed to start a session
    AZStd::intrusive_ptr<XR::GraphicsBindingDescriptor> CreateGraphicsBindingDescriptor()
    {
        return aznew OpenXRVk::SwapChain::View;
    }
}
