/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/base.h>
#include <AzCore/Memory/SystemAllocator.h>

#include <Atom/RHI/XRRenderingInterface.h>
#include <Atom/RPI.Public/XR/XRRenderingInterface.h>
#include <XR/XRInstance.h>
#include <XR/XRDevice.h>
#include <Atom/RHI/ValidationLayer.h>

namespace XR
{
    //! This class is the window to everything XR related. It implements 
    //! RPI::RenderingInterface and RHI::RenderingInterface but
    //! can be extended to implement other non rendering interfaces if needed. 
    class System
        : public AZ::RPI::XRRenderingInterface
        , public AZ::RHI::XRRenderingInterface
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

        ///////////////////////////////////////////////////////////////////
        // AZ::RPI::XRRenderingInterface overrides
        //! Init the XR Instance.
        AZ::RHI::ResultCode InitInstance() override;
        ///////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // AZ::RHI::XRRenderingInterface overrides
        //! Api to init the backend specific native Instance.
        AZ::RHI::ResultCode InitNativeInstance(AZ::RHI::XRInstanceDescriptor* instanceDescriptor) override;
        
        //! Get the number of XR physical devices. 
        AZ::u32 GetNumPhysicalDevices() override;
        
        //! Get the Physical device for a given index.
        AZ::RHI::ResultCode GetXRPhysicalDevice(AZ::RHI::XRPhysicalDeviceDescriptor* physicalDeviceDescriptor, int32_t index) override;
        
        //! API to create XR native device for the renderer.
        AZ::RHI::ResultCode CreateDevice(AZ::RHI::XRDeviceDescriptor* deviceDescriptor) override;
        ///////////////////////////////////////////////////////////////////

    private:
        Ptr<Instance> m_instance;
        Ptr<Device> m_device;
        AZ::RHI::ValidationMode m_validationMode = AZ::RHI::ValidationMode::Disabled;
    };
} // namespace XR
