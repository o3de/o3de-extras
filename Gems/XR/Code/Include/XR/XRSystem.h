/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRSystemInterface.h>

namespace XR
{
    class System
        : public AZ::RPI::XRSystemInterface
    {
    public:
        AZ_RTTI(System, "{C3E0291D-FB30-4E27-AB0D-14606A8C3C1F}" AZ::RPI::XRSystemInterface);

        AZ_DISABLE_COPY_MOVE(System);

        static System* Get();

        // Creates the XR::Instance which is responsible for managing
        // XrInstance (amongst other things) for OpenXR backend
        // Also initializes the XR::Device
        virtual AZ::RHI::ResultCode InitializeSystem() = 0;

        // Create a Session and other basic session-level initialization.
        virtual AZ::RHI::ResultCode InitializeSession() = 0;

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
        virtual AZ::RPI::XRDeviceDescriptor* GetDeviceDescriptor() = 0;

        // Provide access to instance specific data to RHI
        virtual AZ::RPI::XRInstanceDescriptor* GetInstanceDescriptor() = 0;

        // Provide swap chain specific data to RHI
        virtual AZ::RPI::XRSwapChainImageDescriptor* GetSwapChainImageDescriptor(int swapchainIndex) = 0;

        // Provide access to Graphics Binding specific data that RHI can populate
        virtual AZ::RPI::XRGraphicsBindingDescriptor* GetGraphicsBindingDescriptor() = 0;
    };
} // namespace XR
