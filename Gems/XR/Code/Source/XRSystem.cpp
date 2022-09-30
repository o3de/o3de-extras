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

    void System::PostFrame()
    {
        if (m_device && m_session && m_session->IsSessionRunning())
        {
            m_device->PostFrame();
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

    AZ::RHI::ResultCode System::GetViewFov(AZ::u32 viewIndex, AZ::RPI::FovData& outFovData) const
    {
        return m_device->GetViewFov(viewIndex, outFovData);
    }

    AZ::RHI::ResultCode System::GetViewPose(AZ::u32 viewIndex, AZ::RPI::PoseData& outPoseData) const
    {
        return m_device->GetViewPose(viewIndex, outPoseData);
    }

    AZ::RHI::ResultCode System::GetControllerPose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerPose(handIndex, outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    AZ::RHI::ResultCode System::GetControllerStagePose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerStagePose(handIndex, outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    AZ::RHI::ResultCode System::GetViewFrontPose(AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetViewFrontPose(outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    AZ::RHI::ResultCode System::GetViewLocalPose(AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetViewLocalPose(outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    float System::GetControllerScale(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerScale(handIndex);
        }
        return 1.0f;
    }

    float System::GetSqueezeState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetSqueezeState(handIndex);
        }
        return 0.0f;
    }

    float System::GetTriggerState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetTriggerState(handIndex);
        }
        return 0.0f;
    }

    float System::GetXButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetXButtonState();
        }
        return 0.0f;
    }

    float System::GetYButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetYButtonState();
        }
        return 0.0f;
    }

    float System::GetAButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetAButtonState();
        }
        return 0.0f;
    }

    float System::GetBButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetBButtonState();
        }
        return 0.0f;
    }

    float System::GetXJoyStickState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetXJoyStickState(handIndex);
        }
        return 0.0f;
    }

    float System::GetYJoyStickState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetYJoyStickState(handIndex);
        }
        return 0.0f;
    }

    AZ::Matrix4x4 System::CreateStereoscopicProjection(float angleLeft, float angleRight,
                                                 float angleBottom, float angleTop, 
                                                 float nearDist, float farDist, bool reverseDepth)
    {
        return XR::CreateStereoscopicProjection(angleLeft, angleRight, angleBottom, angleTop, nearDist, farDist, reverseDepth);
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
