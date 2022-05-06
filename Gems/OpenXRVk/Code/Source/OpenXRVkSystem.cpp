/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkSystem.h>

namespace OpenXRVk
{
    // Accessor functions for RHI objects that are populated by backend XR gems
    // This will allow XR gem to provide device related data to RHI
    AZ::RPI::XRDeviceDescriptor* OpenXRVk::System::GetDeviceDescriptor()
    {
        return m_deviceDesc.get();
    }

    // Provide access to instance specific data to RHI
    AZ::RPI::XRInstanceDescriptor* OpenXRVk::System::GetInstanceDescriptor()
    {
        return m_instanceDesc.get();
    }

    // Provide Swapchain specific data to RHI
    AZ::RPI::XRSwapChainImageDescriptor* OpenXRVk::System::GetSwapChainImageDescriptor(int swapchainIndex)
    {
        return m_swapchainDesc->m_descriptor.get();
    }

    // Provide access to Graphics Binding specific data that RHI can populate
    AZ::RPI::XRGraphicsBindingDescriptor* OpenXRVk::System::GetGraphicsBindingDescriptor()
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
    AZ::RHI::ResultCode OpenXRVk::System::InitInstance()
    {
        m_instance = XR::Factory::Get()->CreateInstance();

        if (m_instance)
        {
            return m_instance->InitInstanceInternal();
        }
        return AZ::RPI::XR::ResultCode::Fail;
    }

    // Create XR device object and initialize it
    AZ::RHI::ResultCode OpenXRVk::System::InitDevice()
    {
        m_device = XR::Factory::Get()->CreateDevice();

        // Get a list of XR compatible devices
        AZStd::vector<AZStd::intrusive_ptr<PhysicalDevice>> physicalDeviceList = Factory::Get()->EnumerateDeviceList();

        // Code to pick the correct device.
        // For now we can just pick the first device in the list

        if (m_device)
        {
            return m_device->InitDeviceInternal();
        }
        return AZ::RHI::ResultCode::Fail;
    }

    // Initialize XR instance and device
    AZ::RHI::ResultCode OpenXRVk::System::InitializeSystem()
    {
        AZ::RHI::ResultCode instResult = InitInstance();
        if (instResult != AZ::RHI::ResultCode::Success)
        {
            AZ_Assert(false, "XR Instance creation failed");
            return instResult;
        }

        AZ::RHI::ResultCode deviceResult = InitDevice();
        if (deviceResult != AZ::RHI::ResultCode::Success)
        {
            AZ_Assert(false, "XR device creation failed");
            return deviceResult;
        }
        return AZ::RHI::ResultCode::Success;
    }

    // Initialize a XR session
    AZ::RHI::ResultCode OpenXRVk::System::InitializeSession(AZStd::intrusive_ptr<GraphicsBinding> graphicsBinding)
    {
        m_session = XR::Factory::Get()->CreateSession();

        if (m_session)
        {
            XR::SessionDescriptor sessionDesc;
            m_gbDesc = XR::Factory::Get()->CreateGraphicsBindingDescriptor();
            sessionDesc.m_graphicsBinding = RPISystem::Get()->PopulateGrapicsBinding(m_graphicsBindingDesc);
            AZ::RHI::ResultCode sessionResult = m_session->Init(sessionDesc);
            AZ_Assert(sessionResult == AZ::RHI::ResultCode::Success, "Session init failed");

            m_xrInput = XR::Factory::Get()->CreateInput();
            return m_xrInput->InitializeActions();
        }
        return AZ::RHI::ResultCode::Fail;
    }

    // Manage session lifecycle to track if RenderFrame should be called.
    bool OpenXRVk::System::IsSessionRunning() const
    {
        return m_session->IsSessionRunning();
    }

    // Create a Swapchain which will responsible for managing
    // multiple XR swapchains and multiple swapchain images within it
    AZ::RHI::ResultCode OpenXRVk::System::CreateSwapchain()
    {
        m_swapChain = XR::Factory::Get()->CreateSwapchain();

        if (m_swapChain)
        {
            ResultCode swapchainCreationResult = m_swapChain->Init(sessionDesc);
            AZ_Assert(sessionResult == ResultCode::Success, "Swapchain init failed");
            return swapchainCreationResult;
        }
        return AZ::RHI::ResultCode::Fail;
    }

    // Indicate start of a frame
    void OpenXRVk::System::BeginFrame()
    {
    }

    // Indicate end of a frame
    void OpenXRVk::System::EndFrame()
    {
    }

    // Indicate start of a XR view to help with synchronizing XR swapchain
    void OpenXRVk::System::BeginView()
    {
    }

    // Indicate end of a XR view to help with synchronizing XR swapchain
    void OpenXRVk::System::EndView()
    {
    }

    AZ::RHI::ResultCode OpenXRVk::System::InitInstance()
    {
    }

    // System Tick to poll input data
    void OpenXRVk::System::OnSystemTick()
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
}
