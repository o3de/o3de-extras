/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/base.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <Atom/RHI/XRRenderingInterface.h>
#include <Atom/RHI/ValidationLayer.h>
#include <Atom/RPI.Public/XR/XRRenderingInterface.h>
#include <XR/XRInstance.h>
#include <XR/XRDevice.h>
#include <XR/XRSession.h>
#include <XR/XRSwapChain.h>

namespace XR
{
    //! This class is the window to everything XR related. It implements 
    //! RPI::RenderingInterface and RHI::RenderingInterface but
    //! can be extended to implement other non rendering interfaces if needed. 
    class System
        : public AZ::RPI::XRRenderingInterface
        , public AZ::RHI::XRRenderingInterface
        , public AZ::SystemTickBus::Handler
        , public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(System, AZ::SystemAllocator, 0);
        AZ_RTTI(System, "{C3E0291D-FB30-4E27-AB0D-14606A8C3C1F}");

        AZ_DISABLE_COPY_MOVE(System);

        System() = default;
        ~System() = default;

        struct Descriptor
        {
            AZ::RHI::ValidationMode m_validationMode = AZ::RHI::ValidationMode::Disabled;
        };

        //! Init the XRSystem.
        void Init(const Descriptor& descriptor);

        //! Destroy any relevant objects held by this .class 
        void Shutdown();

        //! Handle XR events and actions
        void OnSystemTick() override;

        ///////////////////////////////////////////////////////////////////
        // AZ::RPI::XRRenderingInterface overrides
        AZ::RHI::ResultCode InitInstance() override;
        void AcquireSwapChainImage(AZ::u32 viewIndex) override;
        AZ::u32 GetNumViews() const override;
        AZ::RHI::ResultCode GetSwapChainImage(AZ::RHI::XRSwapChainDescriptor* swapchainDescriptor) const override;
        AZ::u32 GetSwapChainWidth(AZ::u32 viewIndex) const override;
        AZ::u32 GetSwapChainHeight(AZ::u32 viewIndex) const override;
        AZ::RPI::FovData GetViewFov(AZ::u32 viewIndex) const override;
        AZ::RPI::PoseData GetViewPose(AZ::u32 viewIndex) const override;
        AZ::RPI::PoseData GetViewFrontPose() const override;
        AZ::RPI::PoseData GetControllerPose(AZ::u32 handIndex) const override;
        float GetControllerScale(AZ::u32 handIndex) const override;
        bool ShouldRender() const override;
        AZ::Matrix4x4 CreateProjectionOffset(float angleLeft, float angleRight, 
                                             float angleBottom, float angleTop, 
                                             float nearDist, float farDist) override;
        ///////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // AZ::RHI::XRRenderingInterface overrides
        AZ::RHI::ResultCode InitNativeInstance(AZ::RHI::XRInstanceDescriptor* instanceDescriptor) override;
        AZ::u32 GetNumPhysicalDevices() const override;
        AZ::RHI::ResultCode GetXRPhysicalDevice(AZ::RHI::XRPhysicalDeviceDescriptor* physicalDeviceDescriptor, int32_t index) override;
        AZ::RHI::ResultCode CreateDevice(AZ::RHI::XRDeviceDescriptor* deviceDescriptor) override;
        AZ::RHI::ResultCode CreateSession(AZ::RHI::XRSessionDescriptor* sessionDescriptor) override;
        AZ::RHI::ResultCode CreateSwapChain() override;
        AZ::u32 GetCurrentImageIndex(AZ::u32 viewIndex) const override;
        void BeginFrame() override;
        void EndFrame() override;
        ///////////////////////////////////////////////////////////////////

    private:
        Ptr<Instance> m_instance;
        Ptr<Session> m_session;
        Ptr<SwapChain> m_swapChain;
        Ptr<Device> m_device;
        AZ::RHI::ValidationMode m_validationMode = AZ::RHI::ValidationMode::Disabled;
        bool m_isInFrame = false;
    };
} // namespace XR
