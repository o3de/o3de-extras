/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRGraphicsBinding.h>
#include <OpenXRVk_Platform.h>

namespace AZ
{
    namespace OpenXRVk
    {
        class GraphicsBinding final
            : public AZ::RPI::XR::GraphicsBinding
        {
        public:
            class Descriptor final
                : public AZ::RPI::XR::GraphicsBinding::Descriptor
            {
            public:
                XrGraphicsBindingVulkan2KHR m_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR };
            };
        };
    } // namespace OpenXRVk
} // namespace AZ
