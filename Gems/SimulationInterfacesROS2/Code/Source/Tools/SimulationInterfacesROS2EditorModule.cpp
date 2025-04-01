/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesROS2EditorSystemComponent.h"
#include <SimulationInterfacesROS2/SimulationInterfacesROS2TypeIds.h>
#include <SimulationInterfacesROS2ModuleInterface.h>

namespace SimulationInterfacesROS2
{
    class SimulationInterfacesROS2EditorModule : public SimulationInterfacesROS2ModuleInterface
    {
    public:
        AZ_RTTI(SimulationInterfacesROS2EditorModule, SimulationInterfacesROS2EditorModuleTypeId, SimulationInterfacesROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(SimulationInterfacesROS2EditorModule, AZ::SystemAllocator);

        SimulationInterfacesROS2EditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    SimulationInterfacesROS2EditorSystemComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<SimulationInterfacesROS2EditorSystemComponent>(),
            };
        }
    };
} // namespace SimulationInterfacesROS2

AZ_DECLARE_MODULE_CLASS(Gem_SimulationInterfacesROS2_Editor, SimulationInterfacesROS2::SimulationInterfacesROS2EditorModule)
