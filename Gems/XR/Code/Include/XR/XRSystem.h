/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRResult.h>
#include <Atom/RPI.Public/XR/XRDevice.h>
#include <Atom/RPI.Public/XR/XRInstance.h>
#include <Atom/RPI.Public/XR/XRSwapChain.h>
#include <Atom/RPI.Public/XR/XRGraphicsBinding.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            class System
            {
            public:
                AZ_RTTI(System, "{0B08E0D2-FB6C-4290-89BB-556F36CCF50A}");

                AZ_DISABLE_COPY_MOVE(System);

                static System* Get();

                System() = default;
                virtual ~System() = default;

                // Creates the XR::Instance which is responsible for managing
                // XrInstance (amongst other things) for OpenXR backend
                // Also initializes the XR::Device
                virtual ResultCode InitializeSystem() = 0;

                // Create a Session and other basic session-level initialization.
                virtual ResultCode InitializeSession(AZStd::intrusive_ptr<XR::GraphicsBinding> graphicsBinding) = 0;

                // Start of the frame related XR work
                virtual void BeginFrame() = 0;

                // End of the frame related XR work
                virtual void EndFrame() = 0;

                // Start of the XR view related work
                virtual void BeginView() = 0;

                // End of the XR view related work
                virtual void EndView() = 0;

                // Manage session lifecycle to track if RenderFrame should be called.
                virtual bool IsSessionRunning() const = 0;

                // Create a swap chain which will responsible for managing
                // multiple XR swap chains and multiple swap chain images within it
                virtual void CreateSwapchain() = 0;

                // This will allow XR gem to provide device related data to RHI
                virtual Device::Descriptor* GetDeviceDescriptor() = 0;

                // Provide access to instance specific data to RHI
                virtual Instance::Descriptor* GetInstanceDescriptor() = 0;

                // Provide swap chain specific data to RHI
                virtual SwapChain::Image::Descriptor* GetSwapChainImageDescriptor(uint32_t swapchainIndex) = 0;

                // Provide access to Graphics Binding specific data that RHI can populate
                virtual XR::GraphicsBinding::Descriptor* GetGraphicsBindingDescriptor() = 0;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
