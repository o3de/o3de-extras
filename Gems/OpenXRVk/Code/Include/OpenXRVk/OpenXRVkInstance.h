/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/containers/vector.h>
#include <Atom/RHI/ValidationLayer.h>
#include <Atom/RHI.Loader/FunctionLoader.h>
#include <OpenXRVk_Platform.h>
#include <OpenXRVk/OpenXRVkPhysicalDevice.h>
#include <XR/XRInstance.h>

namespace OpenXRVk
{
    //! Vulkan specific XR instance back-end class that will help manage
    //! XR specific vulkan native objects
    class Instance final
    : public XR::Instance
    {
    public:
        AZ_CLASS_ALLOCATOR(Instance, AZ::SystemAllocator, 0);
        AZ_RTTI(Instance, "{1A62DF32-2909-431C-A938-B1E841A50768}", XR::Instance);

        static XR::Ptr<Instance> Create();

        //////////////////////////////////////////////////////////////////////////
        // XR::Instance overrides
        AZ::RHI::ResultCode InitInstanceInternal(AZ::RHI::ValidationMode m_validationMode) override;
        AZ::RHI::ResultCode InitNativeInstance(AZ::RHI::XRInstanceDescriptor* instanceDescriptor) override;
        AZ::u32 GetNumPhysicalDevices() const override;
        AZ::RHI::ResultCode GetXRPhysicalDevice(AZ::RHI::XRPhysicalDeviceDescriptor* physicalDeviceDescriptor, int32_t index) override;
        //////////////////////////////////////////////////////////////////////////

        //! Enumerate supported extension names.
        XR::StringList GetInstanceExtensionNames(const char* layerName = nullptr) const;

        //! Enumerate supported layer names.
        XR::StringList GetInstanceLayerNames() const;

        //! Enumerate and log view configurations.
        void LogViewConfigurations();

        //! Enumerate and log environment blend mode.
        void LogEnvironmentBlendMode(XrViewConfigurationType type);

        //! Get the XRInstance.
        XrInstance GetXRInstance() const;

        //! Get System id.
        XrSystemId GetXRSystemId() const;

        //! Get native VkInstance.
        VkInstance GetNativeInstance() const;

        //! Get glad vulkan context.
        GladVulkanContext& GetContext();

        //! Get function loader.
        AZ::Vulkan::FunctionLoader& GetFunctionLoader();

        //! Get XR environment blend mode.
        XrEnvironmentBlendMode GetEnvironmentBlendMode() const;

        //! Get XR configuration type.
        XrViewConfigurationType GetViewConfigType() const;

        //! Ge the active VkPhysicalDevice.
        VkPhysicalDevice GetActivePhysicalDevice() const;

    private:
        //! Clean native objects.
        void ShutdownInternal() override;

        XrInstance m_xrInstance = XR_NULL_HANDLE;
        VkInstance m_xrVkInstance = VK_NULL_HANDLE;
        XrFormFactor m_formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
        XrViewConfigurationType m_viewConfigType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        XrEnvironmentBlendMode m_environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        XrSystemId m_xrSystemId = XR_NULL_SYSTEM_ID;
        XR::RawStringList m_requiredLayers;
        XR::RawStringList m_requiredExtensions;
        GladVulkanContext m_context;
        AZStd::unique_ptr<AZ::Vulkan::FunctionLoader> m_functionLoader;
        AZStd::vector<VkPhysicalDevice> m_supportedXRDevices;
        AZ::u32 m_physicalDeviceActiveIndex = 0;
    };
}
