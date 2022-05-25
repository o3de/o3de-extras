/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <Atom/RHI/XRRenderingInterface.h>
#include <XR/XRBase.h>
#include <XR/XRInstance.h>
#include <XR/XRObject.h>

namespace XR
{
    class Device
        : public XR::Object
    {
    public:
        AZ_CLASS_ALLOCATOR(Device, AZ::SystemAllocator, 0);
        AZ_RTTI(Device, "{A31B0DC2-BD54-443E-9350-EB1B10670FF9}");

        Device() = default;
        virtual ~Device() = default;

        //////////////////////////////////////////////////////////////////////////
        //! Create the xr specific native device object and populate the XRDeviceDescriptor with it.
        virtual AZ::RHI::ResultCode InitDeviceInternal(AZ::RHI::XRDeviceDescriptor* instanceDescriptor) = 0;
        //////////////////////////////////////////////////////////////////////////
        
        //! Init the XR device.
        AZ::RHI::ResultCode Init(Ptr<Instance> instance);
        
        //! Retrieve the XR instance.
        Ptr<Instance> GetInstance();

    private:

        ///////////////////////////////////////////////////////////////////
        // XR::Object
        void Shutdown() override final;
        ///////////////////////////////////////////////////////////////////

        //! Called when the device is being shutdown.
        virtual void ShutdownInternal() = 0;

        Ptr<Instance> m_instance;
    };
}