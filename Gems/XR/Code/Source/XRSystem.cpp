/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Interface/Interface.h>
#include <AzCore/Debug/Profiler.h>
#include <XR/XRFactory.h>
#include <XR/XRSystem.h>
#include <XR/XRUtils.h>

namespace XR
{
    void System::Init(const System::Descriptor& descriptor)
    {
        m_validationMode = descriptor.m_validationMode;
        AZ::SystemTickBus::Handler::BusConnect();
    }

    AZ::RHI::ResultCode System::InitInstance()
    {
        m_instance = Factory::Get().CreateInstance();

        if (m_instance)
        {
            return m_instance->Init(m_validationMode);
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::InitNativeInstance(AZ::RHI::XRInstanceDescriptor* instanceDescriptor)
    {
        return m_instance->InitNativeInstance(instanceDescriptor);
    }

    AZ::u32 System::GetNumPhysicalDevices() const
    {
        return m_instance->GetNumPhysicalDevices();
    }

    AZ::RHI::ResultCode System::GetXRPhysicalDevice(AZ::RHI::XRPhysicalDeviceDescriptor* physicalDeviceDescriptor, int32_t index)
    {
        AZ_Error("XR", physicalDeviceDescriptor, "The descriptor is null");
        if (physicalDeviceDescriptor)
        {
            return m_instance->GetXRPhysicalDevice(physicalDeviceDescriptor, index);
        }

        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::CreateDevice(AZ::RHI::XRDeviceDescriptor* instanceDescriptor)
    {
        if (!m_device)
        {
            m_device = Factory::Get().CreateDevice();
            AZ_Assert(m_device, "XR Device not created");
            if (m_device->Init(Device::Descriptor{ m_validationMode, m_instance}) == AZ::RHI::ResultCode::Success)
            {
                return m_device->InitDeviceInternal(instanceDescriptor);
            }
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::CreateSession(AZ::RHI::XRSessionDescriptor* sessionDescriptor)
    {
        if (!m_session)
        {
            m_session = Factory::Get().CreateSession();
            AZ_Assert(m_session, "Session not created");
            AZ::RHI::ResultCode result = m_session->Init(Session::Descriptor{ m_validationMode, m_device, m_instance });
            if (result == AZ::RHI::ResultCode::Success)
            {
                return m_session->InitInternal(sessionDescriptor);
            }
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::CreateSwapChain()
    {
        if (!m_swapChain)
        {
            m_swapChain = Factory::Get().CreateSwapChain();
            AZ_Assert(m_swapChain, "XR SwapChain not created");
            return m_swapChain->Init(SwapChain::Descriptor{ m_validationMode, m_instance, m_session, m_device });
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::GetSwapChainImage(AZ::RHI::XRSwapChainDescriptor* swapchainDescriptor) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainImage(swapchainDescriptor);
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::u32 System::GetSwapChainWidth(AZ::u32 viewIndex) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainWidth(viewIndex);
        }
        return 0;
    }

    AZ::u32 System::GetSwapChainHeight(AZ::u32 viewIndex) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainHeight(viewIndex);
        }
        return 0;
    }

    AZ::RHI::Format System::GetSwapChainFormat(AZ::u32 viewIndex) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainFormat(viewIndex);
        }
        return AZ::RHI::Format::Unknown;
    }

    void System::OnSystemTick()
    {
        m_session->PollEvents();
        if (m_session->IsSessionRunning())
        {
            m_session->GetInput()->PollActions();
        }
    }
    
    void System::BeginFrame()
    {
        if (m_device && m_session && m_session->IsSessionRunning())
        {
            m_isInFrame = m_device->BeginFrame();
        }
    }

    void System::EndFrame()
    {
        if (m_isInFrame)
        {
            m_device->EndFrame(m_swapChain);
            m_isInFrame = false;
        }
    }

    void System::AcquireSwapChainImage(AZ::u32 viewIndex)
    {
        if (m_isInFrame && m_device->ShouldRender())
        {
            m_device->AcquireSwapChainImage(viewIndex, m_swapChain.get());
        }
    }

    AZ::u32 System::GetNumViews() const
    {
        return m_swapChain->GetNumViews();
    }

    AZ::u32 System::GetCurrentImageIndex(AZ::u32 viewIndex) const
    {
        SwapChain::View* viewSwapchain = m_swapChain->GetView(viewIndex);
        return viewSwapchain->m_activeImageIndex;
    }

    bool System::ShouldRender() const
    {
        if (m_session->IsSessionRunning())
        { 
            return m_device->ShouldRender();
        }
        return false;
    }

    AZ::RPI::FovData System::GetViewFov(AZ::u32 viewIndex) const
    {
        return m_device->GetViewFov(viewIndex);
    }

    AZ::RPI::PoseData System::GetViewPose(AZ::u32 viewIndex) const
    {
        return m_device->GetViewPose(viewIndex);
    }

    AZ::RPI::PoseData System::GetControllerPose(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerPose(handIndex);
        }
        return AZ::RPI::PoseData();
    }

    float System::GetControllerScale(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerScale(handIndex);
        }
        return 1.0f;
    }

    AZ::RPI::PoseData System::GetViewFrontPose() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetViewFrontPose();
        }
        return AZ::RPI::PoseData();
    }

    AZ::Matrix4x4 System::CreateProjectionOffset(float angleLeft, float angleRight, 
                                                 float angleBottom, float angleTop, 
                                                 float nearDist, float farDist)
    {
        return XR::CreateProjectionOffset(angleLeft, angleRight, angleBottom, angleTop, nearDist, farDist);
    }

    AZ::RHI::XRRenderingInterface* System::GetRHIXRRenderingInterface()
    {
        return this;
    }

    void System::Shutdown()
    {
        AZ::SystemTickBus::Handler::BusDisconnect();
        m_instance = nullptr;
        m_device = nullptr;
    }
}
