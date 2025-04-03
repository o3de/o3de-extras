/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationEntitiesManagerEditor.h"
#include "SimulationFeaturesAggregatorEditor.h"
#include "SimulationManagerEditor.h"
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>
#include <SimulationInterfacesModuleInterface.h>
namespace SimulationInterfaces
{
    class SimulationInterfacesEditorModule : public SimulationInterfacesModuleInterface
    {
    public:
        AZ_RTTI(SimulationInterfacesEditorModule, SimulationInterfacesEditorModuleTypeId, SimulationInterfacesModuleInterface);
        AZ_CLASS_ALLOCATOR(SimulationInterfacesEditorModule, AZ::SystemAllocator);

        SimulationInterfacesEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    SimulationEntitiesManagerEditor::CreateDescriptor(),
                    SimulationManagerEditor::CreateDescriptor(),
                    SimulationFeaturesAggregatorEditor::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<SimulationEntitiesManagerEditor>(),
                azrtti_typeid<SimulationFeaturesAggregatorEditor>(),
            };
        }
    };
} // namespace SimulationInterfaces

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), SimulationInterfaces::SimulationInterfacesEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_SimulationInterfaces_Editor, SimulationInterfaces::SimulationInterfacesEditorModule)
#endif
