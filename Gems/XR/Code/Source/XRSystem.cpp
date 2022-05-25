/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AZCore/Interface/Interface.h>
#include <XR/XRFactory.h>
#include <XR/XRSystem.h>

namespace XR
{
    void System::Init(const System::Descriptor& descriptor)
    {
        m_validationMode = descriptor.m_validationMode;
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

    AZ::u32 System::GetNumPhysicalDevices()
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
            if (m_device->Init(m_instance) == AZ::RHI::ResultCode::Success)
            {
                return m_device->InitDeviceInternal(instanceDescriptor);
            }
        }
        return AZ::RHI::ResultCode::Fail;
    }

    void System::Shutdown()
    {
        m_instance = nullptr;
        m_device = nullptr;
    }
}
