/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/XR/XRSwapChain.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            //! Returns the view swap chain related to the index
            SwapChain::View* SwapChain::GetView(const uint32_t swapchainIndex) const
            {
                return m_viewSwapchains[swapchainIndex].get();
            }

            //! Returns the image associated with the provided image
            //! index and view swap chain index
            SwapChain::Image* SwapChain::GetImage(uint32_t imageIndex, uint32_t swapchainIndex) const
            {
                return GetView(swapchainIndex)->m_images[imageIndex].get();
            }

            ResultCode SwapChain::Init()
            {
                return InitInternal();
            }
        } // namespace XR
    } // namespace RPI
} // namespace AZ
