/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>


namespace XR
{
    //Pull this in when able to test
    /*
    class SwapChainDescriptor 
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(SwapChainDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(SwapChainDescriptor, "{EC10B9A1-9D32-4BC0-AE21-216B1503BBDA}");

        SwapChainDescriptor() = default;
        virtual ~SwapChainDescriptor() = default;

        //any extra info for a generic xr swap chain descriptor
    };

    class SwapChainImageDescriptor
        : public AZ::RPI::XRSwapChainImageDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(SwapChainImageDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(SwapChainImageDescriptor, "{A42444A3-D1E1-489B-8E6F-C0285C7482F3}", AZ::RPI::XRSwapChainImageDescriptor);

        SwapChainImageDescriptor() = default;
        virtual ~SwapChainImageDescriptor() = default;

        //any extra info for a generic xr swap chain
        AZ::u16 m_width;
        AZ::u16 m_height;
        AZ::u16 m_arraySize;
    };

    // This class will be responsible for creating multiple XR::SwapChain::ViewSwapchains
    // (one per view). Each XR::SwapChain::ViewSwapchain will then be responsible
    // for manging and synchronizing multiple swap chain images
    class SwapChain 
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(SwapChain, AZ::SystemAllocator, 0);
        AZ_RTTI(SwapChain, "{0C666E76-E4B7-4097-8D14-713DC2C446EF}");

        SwapChain() = default;
        virtual ~SwapChain() = default;

        class Image 
            : public AZStd::intrusive_base
        {
        public:
            AZ_CLASS_ALLOCATOR(Image, AZ::SystemAllocator, 0);
            AZ_RTTI(Image, "{4037835D-F1BB-4407-BC98-2299CC7BE0A3}");

            Image() = default;
            virtual ~Image() = default;

            Ptr<SwapChainImageDescriptor> m_descriptor;
        };

        class View 
            : public AZStd::intrusive_base
        {
        public:
            AZ_CLASS_ALLOCATOR(View, AZ::SystemAllocator, 0);
            AZ_RTTI(View, "{774EB724-8261-4684-AA78-EDF6BBECD48A}");

            View() = default;
            virtual ~View() = default;

            //! All the images associated with this ViewSwapChain
            AZStd::vector<Ptr<SwapChain::Image>> m_images;

            //! The current image index.
            AZ::u16 m_currentImageIndex = 0;
        };

        //! Returns the view swap chain related to the index
        SwapChain::View* GetView(const AZ::u16 swapChainIndex) const;

        //! Returns the image associated with the provided image
        //! index and view swap chain index
        SwapChain::Image* GetImage(AZ::u16 imageIndex, AZ::u16 swapChainIndex) const;

        AZ::RHI::ResultCode Init();

        virtual AZ::RHI::ResultCode InitInternal();

        Ptr<SwapChainDescriptor> m_descriptor;

    private:
        AZStd::vector<Ptr<SwapChain::View>> m_viewSwapchains;
    };

    */

} // namespace XR
