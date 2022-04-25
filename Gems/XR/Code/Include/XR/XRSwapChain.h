/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRResult.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            // This class will be responsible for creating multiple XR::SwapChain::ViewSwapchains
            // (one per view). Each XR::SwapChain::ViewSwapchain will then be responsible
            // for manging and synchronizing multiple swap chain images
            class SwapChain
            {
            public:
                virtual ~SwapChain() = default;

                class Image
                {
                public:
                    class Descriptor
                    {
                    public:
                        uint16_t m_width;
                        uint16_t m_height;
                        uint16_t m_arraySize;
                    };
                    AZStd::intrusive_ptr<Descriptor> m_descriptor;
                };

                class View
                {
                public:
                    //! All the images associated with this ViewSwapChain
                    AZStd::vector<AZStd::intrusive_ptr<SwapChain::Image>> m_images;

                    //! The current image index.
                    uint32_t m_currentImageIndex = 0;
                };

                //! Returns the view swap chain related to the index
                SwapChain::View* GetView(const uint32_t swapChainIndex) const;

                //! Returns the image associated with the provided image
                //! index and view swap chain index
                SwapChain::Image* GetImage(uint32_t imageIndex, uint32_t swapChainIndex) const;

                ResultCode Init();

                virtual ResultCode InitInternal();

            private:
                AZStd::vector<AZStd::intrusive_ptr<SwapChain::View>> m_viewSwapchains;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
