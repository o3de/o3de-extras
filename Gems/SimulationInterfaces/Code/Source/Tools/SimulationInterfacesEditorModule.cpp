/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LevelManagerEditor.h"
#include "NamedPoseManagerEditor.h"
#include "ROS2SimulationInterfacesEditorSystemComponent.h"
#include "SimulationEntitiesManagerEditor.h"
#include "SimulationFeaturesAggregatorEditor.h"
#include "SimulationManagerEditor.h"
#include <Components/NamedPoseEditorComponent.h>
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
                    NamedPoseManagerEditor::CreateDescriptor(),
                    LevelManagerEditor::CreateDescriptor(),
                    ROS2SimulationInterfaces::ROS2SimulationInterfacesEditorSystemComponent::CreateDescriptor(),
                    NamedPoseEditorComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            // add system components only if application is editor. In other case editor system components break other tools like material
            // Canvas, material editor etc.
            AZ::ApplicationTypeQuery appType;
            AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
            if (appType.IsEditor())
            {
                return AZ::ComponentTypeList{
                    azrtti_typeid<SimulationEntitiesManagerEditor>(),
                    azrtti_typeid<SimulationManagerEditor>(),
                    azrtti_typeid<SimulationFeaturesAggregatorEditor>(),
                    azrtti_typeid<NamedPoseManagerEditor>(),
                    azrtti_typeid<LevelManagerEditor>(),
                    azrtti_typeid<ROS2SimulationInterfaces::ROS2SimulationInterfacesEditorSystemComponent>(),
                };
            }

            return AZ::ComponentTypeList{};
        }
    };
} // namespace SimulationInterfaces

AZ_DECLARE_MODULE_CLASS(Gem_SimulationInterfaces_Editor, SimulationInterfaces::SimulationInterfacesEditorModule)
