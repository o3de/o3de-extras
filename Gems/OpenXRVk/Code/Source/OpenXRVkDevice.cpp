/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkDevice.h>
#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkSession.h>
#include <OpenXRVk/OpenXRVkSwapChain.h>
#include <OpenXRVk/OpenXRVkSpace.h>
#include <OpenXRVk/OpenXRVkUtils.h>
#include <OpenXRVkCommon.h>
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
        Instance* xrVkInstance = static_cast<Instance*>(GetDescriptor().m_instance.get());
        XrVulkanDeviceCreateInfoKHR xrDeviceCreateInfo{ XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR };
        xrDeviceCreateInfo.systemId = xrVkInstance->GetXRSystemId();
        xrDeviceCreateInfo.pfnGetInstanceProcAddr = xrVkInstance->GetContext().GetInstanceProcAddr;
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

        if (GetDescriptor().m_validationMode == AZ::RHI::ValidationMode::Enabled)
        {
            AZ_Printf("OpenXRVk", "Vulkan device extensions to enable: (%i)\n", extensions.size());
            for (const AZStd::string& extension : extensions)
            {
                AZ_Printf("OpenXRVk", "Name=%s\n", extension.c_str());
            }
        }

        VkPhysicalDeviceFeatures features{};
        memcpy(&features, xrDeviceCreateInfo.vulkanCreateInfo->pEnabledFeatures, sizeof(features));

        VkPhysicalDeviceFeatures availableFeatures{};
        xrVkInstance->GetContext().GetPhysicalDeviceFeatures(xrVkInstance->GetActivePhysicalDevice(), &availableFeatures);
        if (availableFeatures.shaderStorageImageMultisample == VK_TRUE)
        {
            // Setting this quiets down a validation error triggered by the Oculus runtime
            features.shaderStorageImageMultisample = VK_TRUE;
        }

        VkDeviceCreateInfo deviceInfo{ VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO };
        memcpy(&deviceInfo, xrDeviceCreateInfo.vulkanCreateInfo, sizeof(deviceInfo));
        deviceInfo.pEnabledFeatures = &features;
        deviceInfo.enabledExtensionCount = aznumeric_cast<AZ::u32>(extensions.size());
        deviceInfo.ppEnabledExtensionNames = extensions.empty() ? nullptr : extensions.data();

        //Create VkDevice
        auto pfnCreateDevice = (PFN_vkCreateDevice)xrDeviceCreateInfo.pfnGetInstanceProcAddr(xrVkInstance->GetNativeInstance(), "vkCreateDevice");
        VkResult vulkanResult = pfnCreateDevice(xrDeviceCreateInfo.vulkanPhysicalDevice, &deviceInfo, xrDeviceCreateInfo.vulkanAllocator, &m_xrVkDevice);
        if (vulkanResult != VK_SUCCESS)
        {
            ShutdownInternal();
            AZ_Error("OpenXRVk", false, "Failed to create the device.");
            return AZ::RHI::ResultCode::Fail;
        }

        {
            VkPhysicalDevice xrVkPhysicalDevice = xrVkInstance->GetActivePhysicalDevice();
            // Now that we have created the device, load the function pointers for it.
            const bool functionsLoaded = xrVkInstance->GetFunctionLoader().LoadProcAddresses(
                &xrVkInstance->GetContext(), xrVkInstance->GetNativeInstance(), xrVkPhysicalDevice, m_xrVkDevice);
            m_context = xrVkInstance->GetContext();

            xrVkInstance->FilterAvailableExtensions(m_context);

            if (!functionsLoaded)
            {
                ShutdownInternal();
                AZ_Error("OpenXRVk", false, "Failed to initialize function loader for the device.");
                return AZ::RHI::ResultCode::Fail;
            }
        }

        //Populate the output data of the descriptor
        xrDeviceDescriptor->m_outputData.m_xrVkDevice = m_xrVkDevice;
        xrDeviceDescriptor->m_outputData.m_context = m_context;

        return AZ::RHI::ResultCode::Success;
    }

    bool Device::BeginFrameInternal()
    {
        Platform::OpenXRBeginFrameInternal();

        Session* session = static_cast<Session*>(GetSession().get());
        XrSession xrSession = session->GetXrSession();

        m_xrLayers.clear();
        m_projectionLayerViews.clear();
        XrFrameWaitInfo frameWaitInfo{ XR_TYPE_FRAME_WAIT_INFO };
        XrResult result = xrWaitFrame(xrSession, &frameWaitInfo, &m_frameState);
        WARN_IF_UNSUCCESSFUL(result);

        XrFrameBeginInfo frameBeginInfo{ XR_TYPE_FRAME_BEGIN_INFO };
        result = xrBeginFrame(xrSession, &frameBeginInfo);
        //The XR_FRAME_DISCARDED can sometimes spam harmlessly so filter it out
        if (result != XR_FRAME_DISCARDED)
        {
            WARN_IF_UNSUCCESSFUL(result);
        }
        //Always return true as we want EndFrame to always be called. 
        return true;
    }

    void Device::EndFrameInternal(XR::Ptr<XR::SwapChain> baseSwapChain)
    {
        Platform::OpenXREndFrameInternal();

        Session* session = static_cast<Session*>(GetSession().get());
        Instance* instance = static_cast<Instance*>(GetDescriptor().m_instance.get());
        SwapChain* swapChain = static_cast<SwapChain*>(baseSwapChain.get());
        Space* xrSpace = static_cast<Space*>(GetSession()->GetSpace());
        XrSession xrSession = session->GetXrSession();

        for(uint32_t i = 0; i < swapChain->GetNumViews(); i++)
        { 
            XR::SwapChain::View* baseSwapChainView = baseSwapChain->GetView(i);
            SwapChain::View* viewSwapChain = static_cast<SwapChain::View*>(baseSwapChainView);

            if (baseSwapChainView->m_isImageAcquired)
            {
                XrSwapchainImageReleaseInfo releaseInfo{ XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
                XrResult result = xrReleaseSwapchainImage(viewSwapChain->GetSwapChainHandle(), &releaseInfo);
                ASSERT_IF_UNSUCCESSFUL(result);

                baseSwapChainView->m_isImageAcquired = false;
            }
        }

        m_xrLayer.space = xrSpace->GetXrSpace(OpenXRVk::SpaceType::View);
        m_xrLayer.viewCount = aznumeric_cast<uint32_t>(m_projectionLayerViews.size());
        m_xrLayer.views = m_projectionLayerViews.data();

        m_xrLayers.push_back(reinterpret_cast<XrCompositionLayerBaseHeader*>(&m_xrLayer));

        XrFrameEndInfo frameEndInfo{ XR_TYPE_FRAME_END_INFO };
        frameEndInfo.displayTime = m_frameState.predictedDisplayTime;

        frameEndInfo.environmentBlendMode = instance->GetEnvironmentBlendMode();
        frameEndInfo.layerCount = aznumeric_cast<uint32_t>(m_xrLayers.size());
        frameEndInfo.layers = m_xrLayers.data();
        XrResult result = xrEndFrame(xrSession, &frameEndInfo);
       
        //The XR_ERROR_VALIDATION_FAILURE can sometimes spam harmlessly so filter it out.
        //It usually happens when xrBeginFrame yields XR_FRAME_DISCARDED 
        if (result != XR_ERROR_VALIDATION_FAILURE)
        {
            WARN_IF_UNSUCCESSFUL(result);
        }
    }

    void Device::PostFrameInternal()
    {
        Platform::OpenXRPostFrameInternal();
    }

    bool Device::AcquireSwapChainImageInternal(AZ::u32 viewIndex, XR::SwapChain* baseSwapChain)
    {
        XR::SwapChain::View* baseSwapChainView = baseSwapChain->GetView(viewIndex);
        SwapChain::View* swapChainView = static_cast<SwapChain::View*>(baseSwapChainView);
        Space* xrSpace = static_cast<Space*>(GetSession()->GetSpace());
        Instance* instance = static_cast<Instance*>(GetDescriptor().m_instance.get());
        Session* session = static_cast<Session*>(GetSession().get());
        XrSession xrSession = session->GetXrSession();
        XrSwapchain swapChainHandle = swapChainView->GetSwapChainHandle();

        XrViewState viewState{ XR_TYPE_VIEW_STATE };
        uint32_t viewCapacityInput = aznumeric_cast<uint32_t>(m_views.size());

        XrViewLocateInfo viewLocateInfo{ XR_TYPE_VIEW_LOCATE_INFO };
        viewLocateInfo.viewConfigurationType = instance->GetViewConfigType(); 
        viewLocateInfo.displayTime = m_frameState.predictedDisplayTime;
        viewLocateInfo.space = xrSpace->GetXrSpace(OpenXRVk::SpaceType::View);

        XrResult result = xrLocateViews(xrSession, &viewLocateInfo, &viewState, viewCapacityInput, &m_viewCountOutput, m_views.data());
        ASSERT_IF_UNSUCCESSFUL(result);
        
        if ((viewState.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) == 0 ||
            (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0)
        {
            //There is no valid tracking poses for the views
            return false;
        }

        AZ_Assert(m_viewCountOutput == viewCapacityInput, "Size mismatch between xrLocateViews %i and xrEnumerateViewConfigurationViews %i", m_viewCountOutput, viewCapacityInput);
        AZ_Assert(m_viewCountOutput == static_cast<SwapChain*>(baseSwapChain)->GetViewConfigs().size(), "Size mismatch between xrLocateViews %i and xrEnumerateViewConfigurationViews %i", m_viewCountOutput, static_cast<SwapChain*>(baseSwapChain)->GetViewConfigs().size());

        m_projectionLayerViews.resize(m_viewCountOutput);
        XrSwapchainImageAcquireInfo acquireInfo{ XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
        result = xrAcquireSwapchainImage(swapChainHandle, &acquireInfo, &baseSwapChainView->m_activeImageIndex);
        baseSwapChainView->m_isImageAcquired = (result == XR_SUCCESS);
        WARN_IF_UNSUCCESSFUL(result);
        
        XrSwapchainImageWaitInfo waitInfo{ XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
        waitInfo.timeout = XR_INFINITE_DURATION;
        result = xrWaitSwapchainImage(swapChainHandle, &waitInfo);
        ASSERT_IF_UNSUCCESSFUL(result);

        m_projectionLayerViews[viewIndex] = { XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW };
        m_projectionLayerViews[viewIndex].pose = m_views[viewIndex].pose;
        m_projectionLayerViews[viewIndex].fov = m_views[viewIndex].fov;
        m_projectionLayerViews[viewIndex].subImage.swapchain = swapChainHandle;
        m_projectionLayerViews[viewIndex].subImage.imageRect.offset = { 0, 0 };
        m_projectionLayerViews[viewIndex].subImage.imageRect.extent = { static_cast<int>(swapChainView->GetWidth()),
                                                                        static_cast<int>(swapChainView->GetHeight()) };
        return true;
    }

    bool Device::ShouldRender() const
    {
        return m_frameState.shouldRender == XR_TRUE;
    }

    void Device::InitXrViews(uint32_t numViews)
    {
        // Create and cache view buffer for xrLocateViews later.
        m_views.clear();
        m_views.resize(numViews, { XR_TYPE_VIEW });
    }

    VkDevice Device::GetNativeDevice() const
    {
        return m_xrVkDevice;
    }

    const GladVulkanContext& Device::GetContext() const
    {
        return m_context;
    }

    AZ::RHI::ResultCode Device::GetViewFov(AZ::u32 viewIndex, AZ::RPI::FovData& outFovData) const
    {
        if(viewIndex < m_projectionLayerViews.size())
        { 
            outFovData.m_angleLeft = m_projectionLayerViews[viewIndex].fov.angleLeft;
            outFovData.m_angleRight = m_projectionLayerViews[viewIndex].fov.angleRight;
            outFovData.m_angleUp = m_projectionLayerViews[viewIndex].fov.angleUp;
            outFovData.m_angleDown = m_projectionLayerViews[viewIndex].fov.angleDown;
            return AZ::RHI::ResultCode::Success;
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode Device::GetViewPose(AZ::u32 viewIndex, AZ::RPI::PoseData& outPoseData) const
    { 
        if (viewIndex < m_projectionLayerViews.size())
        {
            const XrQuaternionf& orientation = m_projectionLayerViews[viewIndex].pose.orientation;
            const XrVector3f& position = m_projectionLayerViews[viewIndex].pose.position;
            outPoseData.m_orientation.Set(orientation.x,
                                          orientation.y, 
                                          orientation.z, 
                                          orientation.w);
            outPoseData.m_position.Set(position.x,
                                       position.y, 
                                       position.z);
            return AZ::RHI::ResultCode::Success;
        }
        return AZ::RHI::ResultCode::Fail;
    }

    XrTime Device::GetPredictedDisplayTime() const
    {
        return m_frameState.predictedDisplayTime;
    }

    void Device::ShutdownInternal()
    {
        m_projectionLayerViews.clear();
        m_views.clear();
        m_xrLayers.clear();
        if (m_xrVkDevice != VK_NULL_HANDLE)
        {
            m_context.DestroyDevice(m_xrVkDevice, nullptr);
            m_xrVkDevice = VK_NULL_HANDLE;
        }
    }
}