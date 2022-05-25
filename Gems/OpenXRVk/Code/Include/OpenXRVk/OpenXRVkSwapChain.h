/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRSwapChain.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    //Pull this in if needed or remove
    /*
    class SwapChainDescriptor final
        : public XR::SwapChainDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(SwapChainDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(SwapChainDescriptor, "{0C6214B3-9271-4972-B6B0-13C4A23D9155}", XR::SwapChainDescriptor);

        SwapChainDescriptor() = default;
        virtual ~SwapChainDescriptor() = default;

        //any extra info for a openxr swap chain descriptor
    };

    class SwapChainImageDescriptor final
        : public XR::SwapChainImageDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(SwapChainImageDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(SwapChainImageDescriptor, "{056D30CF-4B1E-4EC3-9990-A7D9C38C895B}", XR::SwapChainImageDescriptor);

        SwapChainImageDescriptor() = default;
        virtual ~SwapChainImageDescriptor() = default;

        //any extra info for a openxr swap chain image descriptor
    };

    // Class that will help manage XrSwapchain
    class SwapChain final
        : public XR::SwapChain
    {
    public:
        AZ_CLASS_ALLOCATOR(SwapChain, AZ::SystemAllocator, 0);
        AZ_RTTI(SwapChain, "{3DD88236-8C9F-4864-86F5-018C198BC07E}", XR::SwapChain);

        SwapChain() = default;
        virtual ~SwapChain() = default;

        static AZStd::intrusive_ptr<SwapChain> Create();

        class Image final
            : public XR::SwapChain::Image
        {
        public:
            AZ_CLASS_ALLOCATOR(Image, AZ::SystemAllocator, 0);
            AZ_RTTI(Image, "{717ABDD4-C050-4FDF-8E93-3784F81FE315}", XR::SwapChain::Image);

            static AZStd::intrusive_ptr<Image> Create();

        private:
            VkImage m_image;
            XrSwapchainImageBaseHeader* m_swapChainImageHeader;
        };

        class View final
            : public XR::SwapChain::View
        {
        public:
            AZ_CLASS_ALLOCATOR(View, AZ::SystemAllocator, 0);
            AZ_RTTI(View, "{F8312427-AC2D-4737-9A8F-A16ADA5319D0}", XR::SwapChain::View);

            static AZStd::intrusive_ptr<View> Create();
            
            AZ::RHI::ResultCode Init(XrSwapchain handle, AZ::u32 width, AZ::u32 height);

        private:
            XrSwapchain m_handle;
            AZ::u32 m_width;
            AZ::u32 m_height;
        };

        AZ::RHI::ResultCode InitInternal() override;

    private:
        AZStd::vector<XrViewConfigurationView> m_configViews;
        AZStd::vector<XrView> m_views;
        int64_t m_colorSwapchainFormat{ -1 };
    };
    */
}
