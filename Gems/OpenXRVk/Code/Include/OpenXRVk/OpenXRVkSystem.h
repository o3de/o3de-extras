/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/TickBus.h>

#include <XR/XRSystem.h>

#include <OpenXRVk_Platform.h>

#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkDevice.h>
#include <OpenXRVk/OpenXRVkSwapChain.h>
#include <OpenXRVk/OpenXRVkInput.h>
#include <OpenXRVk/OpenXRVkSession.h>
#include <OpenXRVk/OpenXRVkGraphicsBinding.h>

namespace OpenXRVk
{
    class System final
        : public XR::System
        , public AZ::SystemTickBus::Handler
    {
    public:
        AZ_CLASS_ALLOCATOR(System, AZ::SystemAllocator, 0);
        AZ_RTTI(System, "{FBAFDEE2-0A03-4EA8-98E9-A1C8DB32DBCF}", XR::System);

        ///////////////////////////////////////////////////////////////////////////////////
        // SystemInterface
        // Accessor functions for RHI objects that are populated by backend XR gems
        // This will allow XR gem to provide device related data to RHI
        // Initialize XR instance and device
        AZ::RHI::ResultCode InitializeSystem() override;

        // Initialize a XR session
        AZ::RHI::ResultCode InitializeSession() override;

        // Indicate start of a frame
        void BeginFrame() override;

        // Indicate end of a frame
        void EndFrame() override;

        // Indicate start of a XR view to help with synchronizing XR swap chain
        void BeginView() override;

        // Indicate end of a XR view to help with synchronizing XR swap chain
        void EndView() override;

        // Manage session lifecycle to track if RenderFrame should be called.
        bool IsSessionRunning() const override;

        // Create a swap chain which will responsible for managing
        // multiple XR swap chains and multiple swap chain images within it
        AZ::RHI::ResultCode CreateSwapchain() override;

        AZ::RPI::XRDeviceDescriptor* GetDeviceDescriptor() override;

        // Provide access to instance specific data to RHI
        AZ::RPI::XRInstanceDescriptor* GetInstanceDescriptor() override;

        // Provide Swap chain specific data to RHI
        AZ::RPI::XRSwapChainImageDescriptor* GetSwapChainImageDescriptor(AZ::u16 swapchainIndex) override;

        // Provide access to Graphics Binding specific data that RHI can populate
        AZ::RPI::XRGraphicsBindingDescriptor* GetGraphicsBindingDescriptor() override;
        ///////////////////////////////////////////////////////////////////////////////////

    public:
        // Access supported Layers and extension names
        const AZStd::vector<AZStd::string>& GetLayerNames();

        const AZStd::vector<AZStd::string>& GetExtensionNames();

        // Create XR instance object and initialize it
        AZ::RHI::ResultCode InitInstance();

        // Create XR device object and initialize it
        AZ::RHI::ResultCode InitDevice();

    private:

        ///////////////////////////////////////////////////////////////////////////////////
        // SystemTickBus
        // System Tick to poll input data
        void OnSystemTick() override;
        //////////////////////////////////////////////////////////////////////////////////

        AZStd::vector<AZStd::string> m_layerNames;
        AZStd::vector<AZStd::string> m_extentionNames;

        AZStd::intrusive_ptr<OpenXRVk::Instance> m_instance;
        AZStd::intrusive_ptr<OpenXRVk::Device> m_device;
        AZStd::intrusive_ptr<OpenXRVk::Session> m_session;
        AZStd::intrusive_ptr<OpenXRVk::Input> m_input;
        AZStd::intrusive_ptr<OpenXRVk::SwapChain> m_swapChain;
        bool m_requestRestart = false;
        bool m_exitRenderLoop = false;
        AZStd::intrusive_ptr<OpenXRVk::DeviceDescriptor> m_deviceDesc;
        AZStd::intrusive_ptr<OpenXRVk::InstanceDescriptor> m_instanceDesc;
        AZStd::vector<AZStd::intrusive_ptr<OpenXRVk::SwapChainDescriptor>> m_swapchainDesc;
        AZStd::vector<AZStd::intrusive_ptr<OpenXRVk::SwapChainImageDescriptor>> m_swapchainImageDesc;
        AZStd::intrusive_ptr<OpenXRVk::GraphicsBindingDescriptor> m_graphicsBindingDesc;
    };
}
