/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRInstance.h>
#include <Atom/RPI.Public/XR/XRResult.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <glad/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_reflection.h>

namespace AZ
{
    namespace OpenXRVk
    {
        // Class that will help manage XrInstance
        class Instance final
            : public AZ::RPI::XR::Instance
        {
        public:
            static AZStd::intrusive_ptr<Instance> Create();
            AZ::RPI::ResultCode InitInstanceInternal() override;

        private:
            XrInstance m_xrInstance{ XR_NULL_HANDLE };
            AZStd::vector<XrApiLayerProperties> m_layers;
            AZStd::vector<XrExtensionProperties> m_extensions;
            XrFormFactor m_formFactor{ XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY };
            XrSystemId m_systemId{ XR_NULL_SYSTEM_ID };
            VkInstance m_instance = VK_NULL_HANDLE;
        };
    } // namespace OpenXRVk
} // namespace AZ
