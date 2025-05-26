/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationEntitiesManager.h"
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>
#include <SimulationInterfacesModuleInterface.h>

namespace SimulationInterfaces
{
    class SimulationInterfacesModule : public SimulationInterfacesModuleInterface
    {
    public:
        AZ_RTTI(SimulationInterfacesModule, SimulationInterfacesModuleTypeId, SimulationInterfacesModuleInterface);
        AZ_CLASS_ALLOCATOR(SimulationInterfacesModule, AZ::SystemAllocator);
    };
} // namespace SimulationInterfaces

AZ_DECLARE_MODULE_CLASS(Gem_SimulationInterfaces, SimulationInterfaces::SimulationInterfacesModule)
