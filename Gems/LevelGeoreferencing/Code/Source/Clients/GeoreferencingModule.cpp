/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GeoreferencingSystemComponent.h"
#include <Georeferencing/GeoreferencingTypeIds.h>
#include <GeoreferencingModuleInterface.h>
namespace Georeferencing
{
    class GeoreferencingModule : public GeoreferencingModuleInterface
    {
    public:
        AZ_RTTI(GeoreferencingModule, GeoreferencingModuleTypeId, GeoreferencingModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoreferencingModule, AZ::SystemAllocator);
    };
} // namespace Georeferencing

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), Georeferencing::GeoreferencingModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_LevelGeoreferencing, Georeferencing::GeoreferencingModule)
#endif
