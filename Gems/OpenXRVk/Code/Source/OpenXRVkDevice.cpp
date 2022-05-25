/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkDevice.h>
#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkUtils.h>
#include <Atom/RHI.Reflect/Vulkan/XRVkDescriptors.h>
#include <AzCore/Casting/numeric_cast.h>

namespace OpenXRVk
{
    XR::Ptr<Device> Device::Create()
    {
        return aznew Device;
    }

    AZ::RHI::ResultCode Device::InitDeviceInternal(AZ::RHI::XRDeviceDescriptor* deviceDescriptor)
    {
        AZ::Vulkan::XRDeviceDescriptor* xrDeviceDescriptor = static_cast<AZ::Vulkan::XRDeviceDescriptor*>(deviceDescriptor);
        Instance* xrVkInstance = static_cast<Instance*>(GetInstance().get());
        XrVulkanDeviceCreateInfoKHR xrDeviceCreateInfo{ XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR };
        xrDeviceCreateInfo.systemId = xrVkInstance->GetXRSystemId();
        xrDeviceCreateInfo.pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
        xrDeviceCreateInfo.vulkanCreateInfo = xrDeviceDescriptor->m_inputData.m_deviceCreateInfo;
        xrDeviceCreateInfo.vulkanPhysicalDevice = xrVkInstance->GetActivePhysicalDevice();
        xrDeviceCreateInfo.vulkanAllocator = nullptr;

        PFN_xrGetVulkanDeviceExtensionsKHR pfnGetVulkanDeviceExtensionsKHR = nullptr;
        XrResult result = xrGetInstanceProcAddr(
            xrVkInstance->GetXRInstance(), "xrGetVulkanDeviceExtensionsKHR", reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetVulkanDeviceExtensionsKHR));
        ASSERT_IF_UNSUCCESSFUL(result);

        AZ::u32 deviceExtensionNamesSize = 0;
        result = pfnGetVulkanDeviceExtensionsKHR(xrVkInstance->GetXRInstance(), xrDeviceCreateInfo.systemId, 0, &deviceExtensionNamesSize, nullptr);
        ASSERT_IF_UNSUCCESSFUL(result);

        AZStd::vector<char> deviceExtensionNames(deviceExtensionNamesSize);
        result = pfnGetVulkanDeviceExtensionsKHR(
	        xrVkInstance->GetXRInstance(), xrDeviceCreateInfo.systemId, deviceExtensionNamesSize, &deviceExtensionNamesSize, &deviceExtensionNames[0]);
        ASSERT_IF_UNSUCCESSFUL(result);

        AZStd::vector<const char*> extensions = ParseExtensionString(&deviceExtensionNames[0]);
        for (uint32_t i = 0; i < xrDeviceCreateInfo.vulkanCreateInfo->enabledExtensionCount; ++i)
        {
            extensions.push_back(xrDeviceCreateInfo.vulkanCreateInfo->ppEnabledExtensionNames[i]);
        }

        VkPhysicalDeviceFeatures features{};
        memcpy(&features, xrDeviceCreateInfo.vulkanCreateInfo->pEnabledFeatures, sizeof(features));

        VkDeviceCreateInfo deviceInfo{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
        memcpy(&deviceInfo, xrDeviceCreateInfo.vulkanCreateInfo, sizeof(deviceInfo));
        deviceInfo.pEnabledFeatures = &features;
        deviceInfo.enabledExtensionCount = aznumeric_cast<AZ::u32>(extensions.size());
        deviceInfo.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

        //Create VkDevice
        auto pfnCreateDevice = (PFN_vkCreateDevice)xrDeviceCreateInfo.pfnGetInstanceProcAddr(xrVkInstance->GetVkInstance(), "vkCreateDevice");
        VkResult vulkanResult = pfnCreateDevice(xrDeviceCreateInfo.vulkanPhysicalDevice, &deviceInfo, xrDeviceCreateInfo.vulkanAllocator, &m_xrVkDevice);
        if (vulkanResult != VK_SUCCESS)
        {
            return AZ::RHI::ResultCode::Fail;
        }
		
        //Populate the output data of the descriptor
        xrDeviceDescriptor->m_outputData.m_xrVkDevice = m_xrVkDevice;

        return AZ::RHI::ResultCode::Success;
    }

    void Device::ShutdownInternal()
    {
        if (m_xrVkDevice != VK_NULL_HANDLE)
        {
            vkDestroyDevice(m_xrVkDevice, nullptr);
            m_xrVkDevice = VK_NULL_HANDLE;
        }
    }
}
