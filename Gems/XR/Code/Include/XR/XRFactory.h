/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRResult.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <Atom/RPI.Public/XR/XRPhysicalDevice.h>
#include <Atom/RPI.Public/XR/XRInstance.h>
#include <Atom/RPI.Public/XR/XRDevice.h>
#include <Atom/RPI.Public/XR/XRSession.h>
#include <Atom/RPI.Public/XR/XRInput.h>
#include <Atom/RPI.Public/XR/XRSwapChain.h>
#include <Atom/RPI.Public/XR/XRGraphicsBinding.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            //! Interface responsible for creating all the XR objects which are
            //! internally backed by concrete objects
            class Factory
            {
            public:
                AZ_TYPE_INFO(Factory, "{A3D7271A-64FD-442C-9116-DBC32224222F}");

                Factory() = default;
                virtual ~Factory() = default;

                AZ_DISABLE_COPY_MOVE(Factory);

                //! Registers the global factory instance.
                static void Register(AZ::RPI::XR::Factory* instance);

                //! Unregisters the global factory instance.
                static void Unregister(AZ::RPI::XR::Factory* instance);

                //! Access the global factory instance.
                static AZ::RPI::XR::Factory& Get();

                // Create XR::Instance object
                virtual AZStd::intrusive_ptr<AZ::RPI::XR::Instance> CreateInstance() = 0;

                // Create XR::Device object
                virtual AZStd::intrusive_ptr<AZ::RPI::XR::Device> CreateDevice() = 0;

                // Return a list of XR::PhysicalDevice
                virtual AZStd::vector<AZStd::intrusive_ptr<AZ::RPI::XR::PhysicalDevice>> EnumerateDeviceList() = 0;

                // Create XR::Session object
                virtual AZStd::intrusive_ptr<AZ::RPI::XR::Session> CreateSession() = 0;

                // Create XR::Input object
                virtual AZStd::intrusive_ptr<AZ::RPI::XR::Input> CreateInput() = 0;

                // Create XR::SwapChain object
                virtual AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain> CreateSwapchain() = 0;

                // Create XR::ViewSwapChain object
                virtual AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain::View> CreateViewSwapchain() = 0;

                // Create RPI::XR::GraphicsBindingDescriptor that will contain
                // renderer information needed to start a session
                virtual AZStd::intrusive_ptr<AZ::RPI::XR::GraphicsBinding::Descriptor> CreateGraphicsBindingDescriptor() = 0;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
