/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

namespace AZ
{
    namespace OpenXRVk
    {
        /**
        * This module is in charge of loading the RHI reflection descriptor and the
        * system components in charge of managing the different factory backends.
        */
        class Module
            : public AZ::Module
        {
        public:
            AZ_RTTI(PlatformModule, "{C34AA64E-0983-4D30-A33C-0D7C7676A20E}", AZ::Module);
            AZ_CLASS_ALLOCATOR(PlatformModule, AZ::SystemAllocator, 0);

            PlatformModule()
                : AZ::Module()
            {
            }
        };
    }
}

// DO NOT MODIFY THIS LINE UNLESS YOU RENAME THE GEM
// The first parameter should be GemName_GemIdLower
// The second should be the fully qualified name of the class above
AZ_DECLARE_MODULE_CLASS(Gem_OpenXRVk, AZ::OpenXRVk::PlatformModule)
