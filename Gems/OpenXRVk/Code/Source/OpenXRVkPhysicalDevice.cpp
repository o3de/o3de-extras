/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkPhysicalDevice.h>
#include <OpenXRVk/OpenXRVkUtils.h>

namespace OpenXRVk
{
    AZStd::vector<VkPhysicalDevice> PhysicalDevice::EnumerateDeviceList(XrSystemId xrSystemId, XrInstance xrInstance, VkInstance vkInstance)
    {
        AZStd::vector<VkPhysicalDevice> physicalDevices;
		
        XrVulkanGraphicsDeviceGetInfoKHR deviceGetInfo{ XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR };
        deviceGetInfo.systemId = xrSystemId;
        deviceGetInfo.vulkanInstance = vkInstance;
        VkPhysicalDevice vkPhysicalDevice = VK_NULL_HANDLE;

        PFN_xrGetVulkanGraphicsDeviceKHR pfnGetVulkanGraphicsDeviceKHR = nullptr;
        XrResult result = xrGetInstanceProcAddr(
	        xrInstance, "xrGetVulkanGraphicsDeviceKHR", reinterpret_cast<PFN_xrVoidFunction*>(&pfnGetVulkanGraphicsDeviceKHR));
        ASSERT_IF_UNSUCCESSFUL(result);

        //TODO::Look into api that can retreive multiple physicall devices if connected
        result = pfnGetVulkanGraphicsDeviceKHR(xrInstance, xrSystemId, vkInstance, &vkPhysicalDevice);
        ASSERT_IF_UNSUCCESSFUL(result);
        physicalDevices.push_back(vkPhysicalDevice);
        return physicalDevices;
    }
}
