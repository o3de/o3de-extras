/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRGraphicsBinding.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    class GraphicsBindingDescriptor final
        : public XR::GraphicsBindingDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(GraphicsBindingDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(GraphicsBindingDescriptor, "{1083C93E-FB2B-4441-B705-5C44427F2961}", XR::GraphicsBindingDescriptor);

        XrGraphicsBindingVulkan2KHR m_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR };
    };
    
    class GraphicsBinding final
        : public XR::GraphicsBinding
    {
    public:
        AZ_CLASS_ALLOCATOR(GraphicsBinding, AZ::SystemAllocator, 0);
        AZ_RTTI(GraphicsBinding, "{1001E681-EA2E-4898-AC08-B93AA5B63508}", XR::GraphicsBinding);

    };
}
