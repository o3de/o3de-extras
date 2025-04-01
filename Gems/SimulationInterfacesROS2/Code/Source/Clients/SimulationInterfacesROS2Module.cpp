/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesROS2SystemComponent.h"
#include <SimulationInterfacesROS2/SimulationInterfacesROS2TypeIds.h>
#include <SimulationInterfacesROS2ModuleInterface.h>

namespace SimulationInterfacesROS2
{
    class SimulationInterfacesROS2Module : public SimulationInterfacesROS2ModuleInterface
    {
    public:
        AZ_RTTI(SimulationInterfacesROS2Module, SimulationInterfacesROS2ModuleTypeId, SimulationInterfacesROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(SimulationInterfacesROS2Module, AZ::SystemAllocator);
    };
} // namespace SimulationInterfacesROS2

AZ_DECLARE_MODULE_CLASS(Gem_SimulationInterfacesROS2, SimulationInterfacesROS2::SimulationInterfacesROS2Module)
