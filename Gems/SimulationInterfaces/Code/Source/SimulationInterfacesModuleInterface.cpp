/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include "Clients/SimulationFeaturesAggregator.h"
#include "Clients/SimulationManager.h"
#include <Clients/SimulationEntitiesManager.h>

namespace SimulationInterfaces
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        SimulationInterfacesModuleInterface, "SimulationInterfacesModuleInterface", SimulationInterfacesModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(SimulationInterfacesModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(SimulationInterfacesModuleInterface, AZ::SystemAllocator);

    SimulationInterfacesModuleInterface::SimulationInterfacesModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                SimulationEntitiesManager::CreateDescriptor(),
                SimulationManager::CreateDescriptor(),
                SimulationFeaturesAggregator::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList SimulationInterfacesModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<SimulationEntitiesManager>(),
            azrtti_typeid<SimulationManager>(),
            azrtti_typeid<SimulationFeaturesAggregator>(),
        };
    }
} // namespace SimulationInterfaces
