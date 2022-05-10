/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkSystem.h>
#include <OpenXRVk/OpenXRVkFactory.h>

namespace OpenXRVk
{
    // Accessor functions for RHI objects that are populated by backend XR gems
    // This will allow XR gem to provide device related data to RHI
    AZ::RPI::XRDeviceDescriptor* System::GetDeviceDescriptor()
    {
        return m_deviceDesc.get();
    }

    // Provide access to instance specific data to RHI
    AZ::RPI::XRInstanceDescriptor* System::GetInstanceDescriptor()
    {
        return m_instanceDesc.get();
    }

    // Provide Swapchain specific data to RHI
    AZ::RPI::XRSwapChainImageDescriptor* System::GetSwapChainImageDescriptor(AZ::u16 swapchainIndex)
    {
        return m_swapchainImageDesc[swapchainIndex].get();
    }

    // Provide access to Graphics Binding specific data that RHI can populate
    AZ::RPI::XRGraphicsBindingDescriptor* System::GetGraphicsBindingDescriptor()
    {
        return m_graphicsBindingDesc.get();
    }

    // Access supported Layers and extension names
    const AZStd::vector<AZStd::string>& System::GetLayerNames()
    {
        return m_layerNames;
    }

    const AZStd::vector<AZStd::string>& System::GetExtensionNames()
    {
        return m_extentionNames;
    }

    // Create XR instance object and initialize it
    AZ::RHI::ResultCode System::InitInstance()
    {
//         m_instance = Factory::Get().CreateInstance();
// 
//         if (m_instance)
//         {
//             return m_instance->InitInstanceInternal();
//         }
        return AZ::RHI::ResultCode::Fail;
    }

    // Create XR device object and initialize it
    AZ::RHI::ResultCode System::InitDevice()
    {
//         m_device = Factory::Get().CreateDevice();
// 
//         // Get a list of XR compatible devices
//         AZStd::vector<AZStd::intrusive_ptr<PhysicalDevice>> physicalDeviceList = Factory::Get().EnumerateDeviceList();
// 
//         // Code to pick the correct device.
//         // For now we can just pick the first device in the list
// 
//         if (m_device)
//         {
//             return m_device->InitDeviceInternal();
//         }
        return AZ::RHI::ResultCode::Fail;
    }

    // Initialize XR instance and device
    AZ::RHI::ResultCode System::InitializeSystem()
    {
//         AZ::RHI::ResultCode instResult = InitInstance();
//         if (instResult != AZ::RHI::ResultCode::Success)
//         {
//             AZ_Assert(false, "XR Instance creation failed");
//             return instResult;
//         }
// 
//         AZ::RHI::ResultCode deviceResult = InitDevice();
//         if (deviceResult != AZ::RHI::ResultCode::Success)
//         {
//             AZ_Assert(false, "XR device creation failed");
//             return deviceResult;
//         }
        return AZ::RHI::ResultCode::Success;
    }

    // Initialize a XR session
    AZ::RHI::ResultCode System::InitializeSession()
    {
//         m_session = Factory::Get().CreateSession();
// 
//         if (m_session)
//         {
//             XR::SessionDescriptor sessionDesc;
//             m_graphicsBindingDesc = Factory::Get().CreateGraphicsBindingDescriptor();
//             sessionDesc.m_graphicsBinding = RPISystem::Get()->PopulateGrapicsBinding(m_graphicsBindingDesc);
//             AZ::RHI::ResultCode sessionResult = m_session->Init(sessionDesc);
//             AZ_Assert(sessionResult == AZ::RHI::ResultCode::Success, "Session init failed");
// 
//             m_xrInput = Factory::Get().CreateInput();
//             return m_xrInput->InitializeActions();
//         }
        return AZ::RHI::ResultCode::Fail;
    }

    // Manage session lifecycle to track if RenderFrame should be called.
    bool System::IsSessionRunning() const
    {
        return m_session->IsSessionRunning();
    }

    // Create a Swapchain which will responsible for managing
    // multiple XR swapchains and multiple swapchain images within it
    AZ::RHI::ResultCode System::CreateSwapchain()
    {
//         m_swapChain = Factory::Get().CreateSwapchain();
// 
//         if (m_swapChain)
//         {
//             AZ::RHI::ResultCode swapchainCreationResult = m_swapChain->Init(sessionDesc);
//             AZ_Assert(sessionResult == AZ::RHI::ResultCode::Success, "Swapchain init failed");
//             return swapchainCreationResult;
//         }
        return AZ::RHI::ResultCode::Fail;
    }

    // Indicate start of a frame
    void System::BeginFrame()
    {
    }

    // Indicate end of a frame
    void System::EndFrame()
    {
    }

    // Indicate start of a XR view to help with synchronizing XR swapchain
    void System::BeginView()
    {
    }

    // Indicate end of a XR view to help with synchronizing XR swapchain
    void System::EndView()
    {
    }

    // System Tick to poll input data
    void System::OnSystemTick()
    {
//         m_input->PollEvents();
//         if (exitRenderLoop)
//         {
//             break;
//         }
// 
//         if (IsSessionRunning())
//         {
//             m_input->PollActions();
//         }
    }
}
