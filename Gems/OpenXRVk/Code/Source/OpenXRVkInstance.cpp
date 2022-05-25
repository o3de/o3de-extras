/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkUtils.h>
#include <Atom/RHI.Reflect/Vulkan/XRVkDescriptors.h>
#include <AzCore/Casting/numeric_cast.h>

namespace OpenXRVk
{
    AZStd::intrusive_ptr<Instance> Instance::Create()
    {
        return aznew Instance;
    }

    XR::StringList Instance::GetInstanceExtensionNames(const char* layerName /*= nullptr*/) const
    {
        XR::StringList extensionNames;
        AZ::u32 extPropertyCount = 0;
        XrResult result = xrEnumerateInstanceExtensionProperties(layerName, 0, &extPropertyCount, nullptr);
        if (IsError(result) || extPropertyCount == 0)
        {
            return extensionNames;
        }

        AZStd::vector<XrExtensionProperties> extProperties;
        extProperties.resize(extPropertyCount);

        for (XrExtensionProperties& extension : extProperties)
        {
            extension.type = XR_TYPE_EXTENSION_PROPERTIES;
        }

        result = xrEnumerateInstanceExtensionProperties(layerName, aznumeric_cast<AZ::u32>(extProperties.size()), &extPropertyCount, extProperties.data());
        if (IsError(result))
        {
            return extensionNames;
        }

        extensionNames.reserve(extensionNames.size() + extProperties.size());
        for (uint32_t extPropertyIndex = 0; extPropertyIndex < extPropertyCount; extPropertyIndex++)
        {
            extensionNames.emplace_back(extProperties[extPropertyIndex].extensionName);
        }

        return extensionNames;
    }

    XR::StringList Instance::GetInstanceLayerNames() const
    {
        XR::StringList layerNames;
        AZ::u32 layerPropertyCount = 0;
        XrResult result = xrEnumerateApiLayerProperties(0, &layerPropertyCount, nullptr);
        if (IsError(result) || layerPropertyCount == 0)
        {
            return layerNames;
        }

        AZStd::vector<XrApiLayerProperties> layerProperties(layerPropertyCount);
        for (XrApiLayerProperties& layer : layerProperties)
        {
            layer.type = XR_TYPE_API_LAYER_PROPERTIES;
        }

        result = xrEnumerateApiLayerProperties(aznumeric_cast<AZ::u32>(layerProperties.size()), &layerPropertyCount, layerProperties.data());
        if (IsError(result))
        {
            return layerNames;
        }

        layerNames.reserve(layerNames.size() + layerProperties.size());
        for (uint32_t layerPropertyIndex = 0; layerPropertyIndex < layerPropertyCount; ++layerPropertyIndex)
        {
            layerNames.emplace_back(layerProperties[layerPropertyIndex].layerName);
        }

        return layerNames;
    }

    AZ::RHI::ResultCode Instance::InitInstanceInternal(AZ::RHI::ValidationMode validationMode)
    {
        XR::RawStringList optionalLayers = XR::RawStringList{};
        XR::RawStringList optionalExtensions = XR::RawStringList{ { XR_KHR_VULKAN_ENABLE_EXTENSION_NAME } };

        XR::StringList instanceLayerNames = GetInstanceLayerNames();
        XR::RawStringList supportedLayers = FilterList(optionalLayers, instanceLayerNames);
        m_requiredLayers.insert(m_requiredLayers.end(), supportedLayers.begin(), supportedLayers.end());

        XR::StringList instanceExtensions = GetInstanceExtensionNames();
        XR::RawStringList supportedExtensions = FilterList(optionalExtensions, instanceExtensions);
        m_requiredExtensions.insert(m_requiredExtensions.end(), supportedExtensions.begin(), supportedExtensions.end());

        if (validationMode == AZ::RHI::ValidationMode::Enabled)
        {
            AZ_Printf("OpenXrVk", "Available Extensions: (%i)\n", instanceExtensions.size());
            for (const AZStd::string& extension : instanceExtensions)
            {
                AZ_Printf("OpenXrVk", "Name=%s\n", extension.c_str());
            }

            AZ_Printf("OpenXrVk", "Available Layers: (%i)\n", instanceLayerNames.size());
            for (const AZStd::string& layerName : instanceLayerNames)
            {
                AZ_Printf("OpenXrVk", "Name=%s \n", layerName.c_str());
            }
        }

        AZ_Assert(m_xrInstance == XR_NULL_HANDLE, "XR Instance is already initialized");
        XrInstanceCreateInfo createInfo{ XR_TYPE_INSTANCE_CREATE_INFO };
        createInfo.next = nullptr;
        createInfo.enabledExtensionCount = aznumeric_cast<AZ::u32>(supportedExtensions.size());
        createInfo.enabledExtensionNames = supportedExtensions.data();
        createInfo.enabledApiLayerCount = aznumeric_cast<AZ::u32>(supportedLayers.size());
        createInfo.enabledApiLayerNames = supportedLayers.data();

        azstrncpy(createInfo.applicationInfo.applicationName, XR_MAX_APPLICATION_NAME_SIZE, "O3deApp", 4);
        createInfo.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

        //Create XR instance
        XrResult result = xrCreateInstance(&createInfo, &m_xrInstance);
        if(IsError(result))
        {
            AZ_Warning("OpenXrVk", false, "Failed to create XR instance");
            return AZ::RHI::ResultCode::Fail;
        }

        if (validationMode == AZ::RHI::ValidationMode::Enabled)
        {
            XrInstanceProperties instanceProperties{ XR_TYPE_INSTANCE_PROPERTIES };
            result = xrGetInstanceProperties(m_xrInstance, &instanceProperties);
            if (IsSuccess(result))
            {
                AZStd::string verStr = AZStd::string::format("%d.%d.%d",
                XR_VERSION_MAJOR(instanceProperties.runtimeVersion),
                XR_VERSION_MINOR(instanceProperties.runtimeVersion),
                XR_VERSION_PATCH(instanceProperties.runtimeVersion));
                AZ_Printf("OpenXrVk", "Instance RuntimeName=%s RuntimeVersion=%s\n", instanceProperties.runtimeName, verStr.c_str());
            }
        }

        AZ_Assert(m_xrInstance != XR_NULL_HANDLE, "XR Isntance is Null");
        AZ_Assert(m_xrSystemId == XR_NULL_SYSTEM_ID, "XR System id already initialized");

        //TODO::Add support for handheld display
        m_formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

        //TODO::Add support for other view configuration types
        m_viewConfigType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

        //TODO::Add support for other environment blend types
        m_environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;

        XrSystemGetInfo systemInfo{ XR_TYPE_SYSTEM_GET_INFO };
        systemInfo.formFactor = m_formFactor;
        result = xrGetSystem(m_xrInstance, &systemInfo, &m_xrSystemId);
        if (IsError(result))
        {
            AZ_Warning("OpenXrVk", false, "Failed to get XR System id");
            return AZ::RHI::ResultCode::Fail;
        }

        // Query the runtime Vulkan API version requirements
        XrGraphicsRequirementsVulkan2KHR graphicsRequirements{ XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN2_KHR };
        PFN_xrGetVulkanGraphicsRequirementsKHR pfnGetVulkanGraphicsRequirementsKHR = nullptr;
        result = xrGetInstanceProcAddr(m_xrInstance, "xrGetVulkanGraphicsRequirementsKHR",
                                       reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetVulkanGraphicsRequirementsKHR));
        WARN_IF_UNSUCCESSFUL(result);

        XrGraphicsRequirementsVulkanKHR legacyRequirements{ XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR };
        result = pfnGetVulkanGraphicsRequirementsKHR(m_xrInstance, m_xrSystemId, &legacyRequirements);
        WARN_IF_UNSUCCESSFUL(result);
        graphicsRequirements.maxApiVersionSupported = legacyRequirements.maxApiVersionSupported;
        graphicsRequirements.minApiVersionSupported = legacyRequirements.minApiVersionSupported;

        if (validationMode == AZ::RHI::ValidationMode::Enabled)
        {
            AZ_Printf("OpenXrVk", "graphicsRequirements.maxApiVersionSupported %d.%d.%d\n",
            XR_VERSION_MAJOR(graphicsRequirements.maxApiVersionSupported),
            XR_VERSION_MINOR(graphicsRequirements.maxApiVersionSupported),
            XR_VERSION_PATCH(graphicsRequirements.maxApiVersionSupported));

            AZ_Printf("OpenXrVk", "graphicsRequirements.minApiVersionSupported %d.%d.%d\n",
            XR_VERSION_MAJOR(graphicsRequirements.minApiVersionSupported),
            XR_VERSION_MINOR(graphicsRequirements.minApiVersionSupported),
            XR_VERSION_PATCH(graphicsRequirements.minApiVersionSupported));

            AZ_Printf("OpenXrVk", "Using system %d for form factor %s\n", m_xrSystemId, to_string(m_formFactor));
            LogViewConfigurations();
        }
		
        return AZ::RHI::ResultCode::Success;
    }

    void Instance::LogViewConfigurations()
    {
        AZ::u32 viewConfigTypeCount = 0;
        XrResult result = xrEnumerateViewConfigurations(m_xrInstance, m_xrSystemId, 0, &viewConfigTypeCount, nullptr);
        RETURN_IF_UNSUCCESSFUL(result);

        AZStd::vector<XrViewConfigurationType> viewConfigTypes(viewConfigTypeCount);
        result = xrEnumerateViewConfigurations(m_xrInstance, m_xrSystemId, viewConfigTypeCount, &viewConfigTypeCount, viewConfigTypes.data());
        RETURN_IF_UNSUCCESSFUL(result);

        AZ_Warning("OpenXrVk", aznumeric_cast<AZ::u32>(viewConfigTypes.size()) == viewConfigTypeCount, "Size Mismatch");

        AZ_Printf("OpenXrVk", "Available View Configuration Types: (%d)\n", viewConfigTypeCount);
        for (XrViewConfigurationType viewConfigType : viewConfigTypes)
        {
            AZ_Printf("OpenXrVk", "View Configuration Type: %s %s\n", to_string(viewConfigType),
            viewConfigType == m_viewConfigType ? "(Selected)" : "");

            XrViewConfigurationProperties viewConfigProperties{ XR_TYPE_VIEW_CONFIGURATION_PROPERTIES };
            result = xrGetViewConfigurationProperties(m_xrInstance, m_xrSystemId, viewConfigType, &viewConfigProperties);
            RETURN_IF_UNSUCCESSFUL(result);

            AZ_Printf("OpenXrVk", "View configuration FovMutable=%s\n", viewConfigProperties.fovMutable == XR_TRUE ? "True" : "False");

            AZ::u32 viewCount = 0;
            result = xrEnumerateViewConfigurationViews(m_xrInstance, m_xrSystemId, viewConfigType, 0, &viewCount, nullptr);
            RETURN_IF_UNSUCCESSFUL(result);
            if (viewCount > 0)
            {
                AZStd::vector<XrViewConfigurationView> views(viewCount);
                for (XrViewConfigurationView& view : views)
                {
                    view.type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
                }
                result = xrEnumerateViewConfigurationViews(m_xrInstance, m_xrSystemId, viewConfigType, viewCount, &viewCount, views.data());
                RETURN_IF_UNSUCCESSFUL(result);

                for (uint32_t i = 0; i < views.size(); i++)
                {
                    const XrViewConfigurationView& view = views[i];
                    AZ_Printf(
                        "OpenXrVk", "View [%d]: Recommended Width=%d Height=%d SampleCount=%d\n", i, view.recommendedImageRectWidth,
                        view.recommendedImageRectHeight, view.recommendedSwapchainSampleCount);
                    AZ_Printf(
                        "OpenXrVk", "View [%d]:     Maximum Width=%d Height=%d SampleCount=%d\n", i, view.maxImageRectWidth,
                        view.maxImageRectHeight, view.maxSwapchainSampleCount);
                }
            }
            else
            {
                AZ_Printf("OpenXrVk", "Empty view configuration type\n");
            }

            LogEnvironmentBlendMode(viewConfigType);
        }
    }

    void Instance::LogEnvironmentBlendMode(XrViewConfigurationType type)
    {
        AZ::u32 count = 0;
        XrResult result = xrEnumerateEnvironmentBlendModes(m_xrInstance, m_xrSystemId, type, 0, &count, nullptr);
        AZ_Warning("OpenXrVk", count > 0, "BlendModes not supported");
        RETURN_IF_UNSUCCESSFUL(result);

        AZ_Printf("OpenXrVk", "Available Environment Blend Mode count : (%d)\n", count);

        AZStd::vector<XrEnvironmentBlendMode> blendModes(count);
        result = xrEnumerateEnvironmentBlendModes(m_xrInstance, m_xrSystemId, type, count, &count, blendModes.data());
        RETURN_IF_UNSUCCESSFUL(result);

        bool blendModeFound = false;
        for (XrEnvironmentBlendMode mode : blendModes)
        {
            const bool blendModeMatch = (mode == m_environmentBlendMode);
            AZ_Printf("OpenXrVk", "Environment Blend Mode (%s) : %s\n", to_string(mode), blendModeMatch ? "(Selected)" : "");
            blendModeFound |= blendModeMatch;
        }
    }

    AZ::RHI::ResultCode Instance::InitNativeInstance(AZ::RHI::XRInstanceDescriptor* instanceDescriptor)
    {
        m_functionLoader = FunctionLoader::Create();
        if (!m_functionLoader->Init())
        {
            AZ_Warning("Vulkan", false, "OpenXRVk Could not initialized function loader.");
            return AZ::RHI::ResultCode::Fail;
        }

        AZ::Vulkan::XRInstanceDescriptor* xrInstanceDescriptor = static_cast<AZ::Vulkan::XRInstanceDescriptor*>(instanceDescriptor);
        XrVulkanInstanceCreateInfoKHR createInfo{ XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR };
        createInfo.systemId = m_xrSystemId;
        createInfo.pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
        createInfo.vulkanCreateInfo = xrInstanceDescriptor->m_inputData.m_createInfo;
        createInfo.vulkanAllocator = nullptr;

        PFN_xrGetVulkanInstanceExtensionsKHR pfnGetVulkanInstanceExtensionsKHR = nullptr;
        XrResult result = xrGetInstanceProcAddr(m_xrInstance, "xrGetVulkanInstanceExtensionsKHR", reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetVulkanInstanceExtensionsKHR));
        ASSERT_IF_UNSUCCESSFUL(result);

        AZ::u32 extensionNamesSize = 0;
        result = pfnGetVulkanInstanceExtensionsKHR(m_xrInstance, m_xrSystemId, 0, &extensionNamesSize, nullptr);
        ASSERT_IF_UNSUCCESSFUL(result);

        AZStd::vector<char> extensionNames(extensionNamesSize);
        result = pfnGetVulkanInstanceExtensionsKHR(m_xrInstance, m_xrSystemId, extensionNamesSize, &extensionNamesSize, &extensionNames[0]);
        ASSERT_IF_UNSUCCESSFUL(result);
		
        AZStd::vector<const char*> extensions = ParseExtensionString(&extensionNames[0]);
        for (uint32_t i = 0; i < createInfo.vulkanCreateInfo->enabledExtensionCount; ++i)
        {
            extensions.push_back(createInfo.vulkanCreateInfo->ppEnabledExtensionNames[i]);
        }

        VkInstanceCreateInfo instInfo{ VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO };
        memcpy(&instInfo, createInfo.vulkanCreateInfo, sizeof(instInfo));
        instInfo.enabledExtensionCount = aznumeric_cast<AZ::u32>(extensions.size());
        instInfo.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

        auto pfnCreateInstance = (PFN_vkCreateInstance)createInfo.pfnGetInstanceProcAddr(nullptr, "vkCreateInstance");
        VkResult vkResult = pfnCreateInstance(&instInfo, nullptr, &m_xrVkInstance);
        if (vkResult != VK_SUCCESS)
        {
            return AZ::RHI::ResultCode::Fail;
        }
		
        //Populate the instance descriptor with the correct VkInstance
        xrInstanceDescriptor->m_outputData.m_xrVkInstance = m_xrVkInstance;

        //Get the list of Physical devices
        m_supportedXRDevices = PhysicalDevice::EnumerateDeviceList(m_xrSystemId, m_xrInstance, m_xrVkInstance);
        if (m_supportedXRDevices.size() > 1)
        {
            //Just use the first device at the moment.
            m_physicalDeviceActiveIndex = 0;
        }
        return AZ::RHI::ResultCode::Success;
    }

    void Instance::ShutdownInternal()
    {
        if (m_xrVkInstance != VK_NULL_HANDLE)
        {
            m_supportedXRDevices.clear();

            vkDestroyInstance(m_xrVkInstance, nullptr);
            m_functionLoader = VK_NULL_HANDLE;
        }

        if (m_functionLoader)
        {
            m_functionLoader->Shutdown();
        }
        m_functionLoader = nullptr;
    }

    AZ::RHI::ResultCode Instance::GetXRPhysicalDevice(AZ::RHI::XRPhysicalDeviceDescriptor* physicalDeviceDescriptor, int32_t index)
    {
        AZ::Vulkan::XRPhysicalDeviceDescriptor* xrPhysicalDeviceDescriptor = static_cast<AZ::Vulkan::XRPhysicalDeviceDescriptor*>(physicalDeviceDescriptor);
        if (xrPhysicalDeviceDescriptor && (index < m_supportedXRDevices.size()))
        {
            xrPhysicalDeviceDescriptor->m_outputData.m_xrVkPhysicalDevice = m_supportedXRDevices[index];
            return AZ::RHI::ResultCode::Success;
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::u32 Instance::GetNumPhysicalDevices()
    {
        return aznumeric_cast<AZ::u32>(m_supportedXRDevices.size());
    }

    VkPhysicalDevice Instance::GetActivePhysicalDevice()
    {
        AZ_Assert(m_physicalDeviceActiveIndex < m_supportedXRDevices.size(), "Index out of range");
        return m_supportedXRDevices[m_physicalDeviceActiveIndex];
    }

    XrInstance Instance::GetXRInstance()
    {
        return m_xrInstance;
    }

    XrSystemId Instance::GetXRSystemId()
    {
        return m_xrSystemId;
    }

    VkInstance Instance::GetVkInstance()
    {
        return m_xrVkInstance;
    }

    XrEnvironmentBlendMode Instance::GetEnvironmentBlendMode()
    {
        return m_environmentBlendMode;
    }

    XrViewConfigurationType Instance::GetViewConfigType()
    {
        return m_viewConfigType;
    }
}
