/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkSystem.h>

namespace AZ
{
    namespace OpenXRVk
    {
        // Accessor functions for RHI objects that are populated by backend XR gems
        // This will allow XR gem to provide device related data to RHI
        AZ::RPI::XR::Device::Descriptor* OpenXRVk::System::GetDeviceDescriptor()
        {
            return m_deviceDesc.get();
        }

        // Provide access to instance specific data to RHI
        RPI::XR::Instance::Descriptor* OpenXRVk::System::GetInstanceDescriptor()
        {
            return m_instanceDesc.get();
        }

        // Provide Swapchain specific data to RHI
        RPI::XR::SwapChain::Image::Descriptor* OpenXRVk::System::GetSwapChainImageDescriptor(int swapchainIndex)
        {
            return m_swapchainDesc->m_descriptor.get();
        }

        // Provide access to Graphics Binding specific data that RHI can populate
        RHI::GraphicsBinding::Descriptor* OpenXRVk::System::GetGraphicsBindingDescriptor()
        {
            return m_graphicsBindingDesc.get();
        }

        // Access supported Layers and extension names
        const AZStd::vector<AZStd::string>& OpenXRVk::System::GetLayerNames()
        {
        }

        const AZStd::vector<AZStd::string>& OpenXRVk::System::GetExtensionNames()
        {
        }

        // Create XR instance object and initialize it
        AZ::RPI::XR::ResultCode OpenXRVk::System::InitInstance()
        {
            m_instance = Factory::Get()->CreateXRInstance();

            if (m_instance)
            {
                return m_instance->InitInstanceInternal();
            }
            return AZ::RPI::XR::ResultCode::Fail;
        }

        // Create XR device object and initialize it
        AZ::RPI::XR::ResultCode OpenXRVk::System::InitDevice()
        {
            m_device = Factory::Get()->CreateXRDevice();

            // Get a list of XR compatible devices
            AZStd::vector<AZStd::intrusive_ptr<PhysicalDevice>> physicalDeviceList = Factory::Get()->EnumerateDeviceList();

            // Code to pick the correct device.
            // For now we can just pick the first device in the list

            if (m_device)
            {
                return m_device->InitDeviceInternal();
            }
            return AZ::RPI::XR::ResultCode::Fail;
        }

        // Initialize XR instance and device
        AZ::RPI::XR::ResultCode OpenXRVk::System::InitializeSystem() override
        {
            AZ::RPI::XR::ResultCode instResult = InitInstance();
            if (instResult != AZ::RPI::XR::ResultCode::Success)
            {
                AZ_Assert(false, "XR Instance creation failed");
                return instResult;
            }

            AZ::RPI::XR::ResultCode deviceResult = InitDevice();
            if (deviceResult != AZ::RPI::XR::ResultCode::Success)
            {
                AZ_Assert(false, "XR device creation failed");
                return deviceResult;
            }
            return AZ::RPI::XR::ResultCode::Success;
        }

        // Initialize a XR session
        AZ::RPI::XR::ResultCode OpenXRVk::System::InitializeSession(AZStd::intrusive_ptr<GraphicsBinding> graphicsBinding) override
        {
            m_session = Factory::Get()->CreateXRSession();

            if (m_session)
            {
                AZ::RPI::XR::Session::Descriptor sessionDesc;
                m_gbDesc = Factory::Get()->CreateGraphicsBindingDescriptor();
                sessionDesc.m_graphicsBinding = RPISystem::Get()->PopulateGrapicsBinding(m_graphicsBindingDesc);
                ResultCode sessionResult = m_session->Init(sessionDesc);
                AZ_Assert(sessionResult == ResultCode::Success, "Session init failed");

                m_xrInput = Factory::Get()->CreateXRInput();
                return m_xrInput->InitializeActions();
            }
            return AZ::RPI::XR::ResultCode::Fail;
        }

        // Manage session lifecycle to track if RenderFrame should be called.
        bool OpenXRVk::System::IsSessionRunning() const override
        {
            return m_session->IsSessionRunning();
        }

        // Create a Swapchain which will responsible for managing
        // multiple XR swapchains and multiple swapchain images within it
        AZ::RPI::XR::ResultCode OpenXRVk::System::CreateSwapchain() override
        {
            m_swapChain = Factory::Get()->CreateSwapchain();

            if (m_swapChain)
            {
                ResultCode swapchainCreationResult = m_swapChain->Init(sessionDesc);
                AZ_Assert(sessionResult == ResultCode::Success, "Swapchain init failed");
                return swapchainCreationResult;
            }
            return AZ::RPI::XR::ResultCode::Fail;
        }

        // Indicate start of a frame
        void OpenXRVk::System::BeginFrame() override
        {
        }

        // Indicate end of a frame
        void OpenXRVk::System::EndFrame() override
        {
        }

        // Indicate start of a XR view to help with synchronizing XR swapchain
        void OpenXRVk::System::BeginXRView() override
        {
        }

        // Indicate end of a XR view to help with synchronizing XR swapchain
        void OpenXRVk::System::EndXRView() override
        {
        }

        AZ::RPI::XR::ResultCode OpenXRVk::System::InitInstance()
        {
        }

        // System Tick to poll input data
        void OpenXRVk::System::OnSystemTick() override
        {
            m_input->PollEvents();
            if (exitRenderLoop)
            {
                break;
            }

            if (IsSessionRunning())
            {
                m_input->PollActions();
            }
        }
    } // namespace OpenXRVk
} // namespace AZ
