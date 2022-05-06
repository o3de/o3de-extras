/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace XR
{
    class Module
        : public AZ::Module
    {
    public:
        AZ_RTTI(Module, "{71A99524-7D31-42D3-955E-3F4774F310AC}", AZ::Module);
        AZ_CLASS_ALLOCATOR(Module, AZ::SystemAllocator, 0);

        Module()
            : AZ::Module()
        {
        }

    };
}

// DO NOT MODIFY THIS LINE UNLESS YOU RENAME THE GEM
// The first parameter should be GemName_GemIdLower
// The second should be the fully qualified name of the class above
AZ_DECLARE_MODULE_CLASS(Gem_XR, XR::Module)
