/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkSwapChain.h>

namespace AZ
{
    namespace OpenXRVk
    {
        AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain> SwapChain::Create()
        {
        }

        AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain::Image> SwapChain::Image::Create()
        {
        }

        AZStd::intrusive_ptr<AZ::RPI::XR::SwapChain::View> SwapChain::View::Create()
        {
        }

        AZ::RPI::XR::ResultCode SwapChain::View::Init(XrSwapchain handle, uint32_t width, uint32_t height)
        {
            return AZ::RPI::XR::ResultCode::Success;
        }

        AZ::RPI::XR::ResultCode SwapChain::InitInternal()
        {
            // xrEnumerateViewConfigurationViews
            for (int i = 0; i < m_views.size(); i++)
            {
                // xrCreateSwapchain
                AZStd::intrusive_ptr<SwapChain::View> vSwapChain = Factory::Get()->ViewSwapChain();

                if (vSwapChain)
                {
                    xrCreateSwapchain(.., xrSwapchainHandle, .) vSwapChain->Init(xrSwapchainHandle, ..);
                    m_viewSwapchains.push_back(vSwapChain);
                }
            }
            return AZ::RPI::XR::ResultCode::Success;
        }
    } // namespace OpenXRVk
} // namespace AZ
