/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>

#include <Atom/RPI.Public/XR/XRSystemInterface.h>

namespace XR
{
    class GraphicsBindingDescriptor
        : public AZ::RPI::XRGraphicsBindingDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(GraphicsBindingDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(GraphicsBindingDescriptor, "{6027891C-8920-4B36-83B6-FDF4E3DDDEC7}");

        GraphicsBindingDescriptor() = default;
        virtual ~GraphicsBindingDescriptor() = default;

        //any extra info for a generic xr GraphicsBindingDescriptor
    };

    class GraphicsBinding
    {
    public:
        AZ_CLASS_ALLOCATOR(GraphicsBinding, AZ::SystemAllocator, 0);
        AZ_RTTI(GraphicsBinding, "{0520401C-26B2-49E0-8EFD-6AD8E0720E84}");

        GraphicsBinding() = default;
        virtual ~GraphicsBinding() = default;

        AZStd::intrusive_ptr<GraphicsBindingDescriptor> m_descriptor;
    };
} // namespace XR

