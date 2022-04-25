/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRSwapChain.h>
#include <glad/vulkan.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_reflection.h>

namespace AZ
{
    namespace OpenXRVk
    {
        // Class that will help manage XrSwapchain
        class SwapChain final
            : public AZ::RPI::XR::SwapChain
        {
        public:
            static AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain> Create();

            class Image final
                : public AZ::RPI::XR::SwapChain::Image
            {
            public:
                class Descriptor final
                    : public AZ::RPI::XR::SwapChain::Image::Descriptor
                {
                public:
                };
                static AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain::Image> Create();

            private:
                VkImage m_image;
                XrSwapchainImageBaseHeader* m_swapChainImageHeader;
            };

            class View final
                : public AZ::RPI::XR::SwapChain::View
            {
            public:
                static AZStd::intrusive_ptr<View> Create();
                AZ::RPI::XR::ResultCode Init(XrSwapchain handle, uint32_t width, uint32_t height);

            private:
                XrSwapchain m_handle;
                int32_t m_width;
                int32_t m_height;
            };

            AZ::RPI::XR::ResultCode InitInternal() override;

        private:
            AZStd::vector<XrViewConfigurationView> m_configViews;
            AZStd::vector<XrView> m_views;
            int64_t m_colorSwapchainFormat{ -1 };
        };
    } // namespace OpenXRVk
} // namespace AZ
