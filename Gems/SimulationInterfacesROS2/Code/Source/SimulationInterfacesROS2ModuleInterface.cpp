/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesROS2ModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <SimulationInterfacesROS2/SimulationInterfacesROS2TypeIds.h>

#include <Clients/SimulationInterfacesROS2SystemComponent.h>

namespace SimulationInterfacesROS2
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        SimulationInterfacesROS2ModuleInterface, "SimulationInterfacesROS2ModuleInterface", SimulationInterfacesROS2ModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(SimulationInterfacesROS2ModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(SimulationInterfacesROS2ModuleInterface, AZ::SystemAllocator);

    SimulationInterfacesROS2ModuleInterface::SimulationInterfacesROS2ModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                SimulationInterfacesROS2SystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList SimulationInterfacesROS2ModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<SimulationInterfacesROS2SystemComponent>(),
        };
    }
} // namespace SimulationInterfacesROS2
