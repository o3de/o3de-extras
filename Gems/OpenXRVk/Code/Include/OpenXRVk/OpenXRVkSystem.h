/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RHI/GraphicsBinding.h>
#include <Atom/RPI.Public/XR/XRSystemInterface.h>
#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkDevice.h>
#include <OpenXRVk/OpenXRVkSwapChain.h>
#include <OpenXRVk/OpenXRVkInput.h>
#include <OpenXRVk/OpenXRVkSession.h>
#include <OpenXRVk/OpenXRVkGraphicsBinding.h>

#include <AzCore/Component/TickBus.h>

namespace AZ
{
    namespace OpenXRVk
    {
        class System final
            : public AZ::RPI::XR::SystemInterface
            , public AZ::SystemTickBus::Handler
        {
        public:
            AZ_RTTI(System, "{FBAFDEE2-0A03-4EA8-98E9-A1C8DB32DBCF}", SystemInterface);

            virtual ~System() = default;

            ///////////////////////////////////////////////////////////////////////////////////
            // SystemInterface
            // Accessor functions for RHI objects that are populated by backend XR gems
            // This will allow XR gem to provide device related data to RHI
            // Initialize XR instance and device
            AZ::RPI::XR::ResultCode InitializeSystem() override;

            // Initialize a XR session
            AZ::RPI::XR::ResultCode InitializeSession(AZStd::intrusive_ptr<AZ::RPI::XR::GraphicsBinding> graphicsBinding) override;

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
            AZ::RPI::XR::ResultCode CreateSwapchain() override;

            AZ::RPI::XR::Device::Descriptor* GetDeviceDescriptor() override;

            // Provide access to instance specific data to RHI
            AZ::RPI::XR::Instance::Descriptor* GetInstanceDescriptor() override;

            // Provide Swap chain specific data to RHI
            AZ::RPI::XR::SwapChain::Image::Descriptor* GetSwapChainImageDescriptor(int swapchainIndex) override;

            // Provide access to Graphics Binding specific data that RHI can populate
            AZ::RHI::GraphicsBinding::Descriptor* GetGraphicsBindingDescriptor() override;
            ///////////////////////////////////////////////////////////////////////////////////

        public:
            // Access supported Layers and extension names
            const AZStd::vector<AZStd::string>& GetLayerNames();

            const AZStd::vector<AZStd::string>& GetExtensionNames();

            // Create XR instance object and initialize it
            AZ::RPI::XR::ResultCode InitInstance();

            // Create XR device object and initialize it
            AZ::RPI::XR::ResultCode InitDevice();

        private:
            AZ::RPI::XR::ResultCode InitInstance();

            ///////////////////////////////////////////////////////////////////////////////////
            // SystemTickBus
            // System Tick to poll input data
            void OnSystemTick() override;
            //////////////////////////////////////////////////////////////////////////////////

            AZStd::intrusive_ptr<OpenXRVk::Instance> m_instance;
            AZStd::intrusive_ptr<OpenXRVk::Device> m_device;
            AZStd::intrusive_ptr<OpenXRVk::Session> m_session;
            AZStd::intrusive_ptr<OpenXRVk::Input> m_input;
            AZStd::intrusive_ptr<OpenXRVk::SwapChain> m_swapChain;
            bool m_requestRestart = false;
            bool m_exitRenderLoop = false;
            AZStd::intrusive_ptr<OpenXRVk::Device::Descriptor> m_deviceDesc;
            AZStd::intrusive_ptr<OpenXRVk::Instance::Descriptor> m_instanceDesc;
            AZStd::intrusive_ptr<OpenXRVk::SwapChain::Descriptor> m_swapchainDesc;
            AZStd::intrusive_ptr<OpenXRVk::GraphicsBinding::Descriptor> m_graphicsBindingDesc;
        };
    } // namespace OpenXRVk
} // namespace AZ
