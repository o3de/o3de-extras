/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRInstance.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    class InstanceDescriptor final
        : public XR::InstanceDescriptor
    {
    public:
        AZ_RTTI(InstanceDescriptor, "{F7D29A7A-5841-4B6F-ADFE-3734316BC63D}", XR::InstanceDescriptor);

        InstanceDescriptor() = default;
        virtual ~InstanceDescriptor() = default;

        //any extra info a openxr instance descriptor needs
    };

    // Class that will help manage XrInstance
    class Instance final
        : public XR::Instance
    {
    public:
        AZ_RTTI(Instance, "{1A62DF32-2909-431C-A938-B1E841A50768}", XR::Instance);

        Instance() = default;
        virtual ~Instance() = default;

        static AZStd::intrusive_ptr<Instance> Create();
        virtual AZ::RHI::ResultCode InitInstanceInternal() override;

    private:
        XrInstance m_xrInstance{ XR_NULL_HANDLE };
        AZStd::vector<XrApiLayerProperties> m_layers;
        AZStd::vector<XrExtensionProperties> m_extensions;
        XrFormFactor m_formFactor{ XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY };
        XrSystemId m_systemId{ XR_NULL_SYSTEM_ID };
        VkInstance m_instance = VK_NULL_HANDLE;
    };
}
