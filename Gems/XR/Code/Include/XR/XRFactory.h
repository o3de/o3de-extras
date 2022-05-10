/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <AzCore/Interface/Interface.h>

#include <XR/XRPhysicalDevice.h>
#include <XR/XRInstance.h>
#include <XR/XRDevice.h>
#include <XR/XRSession.h>
#include <XR/XRInput.h>
#include <XR/XRSwapChain.h>
#include <XR/XRGraphicsBinding.h>

namespace XR
{
    //! Interface responsible for creating all the XR objects which are
    //! internally backed by concrete objects
    class Factory
    {
    public:
        AZ_CLASS_ALLOCATOR(Factory, AZ::SystemAllocator, 0);
        AZ_RTTI(Factory, "{A3D7271A-64FD-442C-9116-DBC32224222F}");

        Factory() = default;
        virtual ~Factory() = default;

        AZ_DISABLE_COPY_MOVE(Factory);

        //! Registers the global factory instance.
        static void Register(XR::Factory* instance);

        //! Unregisters the global factory instance.
        static void Unregister(XR::Factory* instance);

        //! Is the factory ready
        static bool IsReady();

        //! Access the global factory instance.
        static XR::Factory& Get();

        // Create XR::Instance object
        virtual AZStd::intrusive_ptr<XR::Instance> CreateInstance() = 0;

        // Create XR::Device object
        virtual AZStd::intrusive_ptr<XR::Device> CreateDevice() = 0;

        // Return a list of XR::PhysicalDevice
        virtual AZStd::vector<AZStd::intrusive_ptr<XR::PhysicalDevice>> EnumerateDeviceList() = 0;

        // Create XR::Session object
        virtual AZStd::intrusive_ptr<XR::Session> CreateSession() = 0;

        // Create XR::Input object
        virtual AZStd::intrusive_ptr<XR::Input> CreateInput() = 0;

        // Create XR::SwapChain object
        virtual AZStd::intrusive_ptr<XR::SwapChain> CreateSwapchain() = 0;

        // Create XR::ViewSwapChain object
        virtual AZStd::intrusive_ptr<XR::SwapChain::View> CreateViewSwapchain() = 0;

        // Create RPI::XR::GraphicsBindingDescriptor that will contain
        // renderer information needed to start a session
        virtual AZStd::intrusive_ptr<XR::GraphicsBindingDescriptor> CreateGraphicsBindingDescriptor() = 0;
    };
} // namespace XR

