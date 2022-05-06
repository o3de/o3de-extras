/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <XR/XRSwapChain.h>

namespace XR
{
    //! Returns the view swap chain related to the index
    SwapChain::View* SwapChain::GetView(const AZ::u16 swapchainIndex) const
    {
        return m_viewSwapchains[swapchainIndex].get();
    }

    //! Returns the image associated with the provided image
    //! index and view swap chain index
    SwapChain::Image* SwapChain::GetImage(AZ::u16 imageIndex, AZ::u16 swapchainIndex) const
    {
        return GetView(swapchainIndex)->m_images[imageIndex].get();
    }

    AZ::RHI::ResultCode SwapChain::Init()
    {
        return InitInternal();
    }
} // namespace XR
