/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RHI/GraphicsBinding.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            class GraphicsBinding
                : public AZ::RHI::GraphicsBinding
            {
            public:
                GraphicsBinding() = default;
                ~GraphicsBinding() = default;

                class Descriptor
                    : public AZ::RHI::GraphicsBinding::Descriptor
                {
                public:
                    Descriptor() = default;
                    ~Descriptor() = default;
                };
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
