/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationEntitiesMangerEditor.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    AZ_COMPONENT_IMPL(SimulationEntitiesMangerEditor,
        "SimulationEntitiesMangerEditor",
        SimulationEntitiesManagerEditorTypeId,
        BaseSystemComponent);

    void SimulationEntitiesMangerEditor::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationEntitiesMangerEditor, SimulationEntitiesManager>()->Version(0);
        }
    }

    SimulationEntitiesMangerEditor::SimulationEntitiesMangerEditor() = default;

    SimulationEntitiesMangerEditor::~SimulationEntitiesMangerEditor() = default;

    void SimulationEntitiesMangerEditor::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SimulationInterfacesEditorService"));
    }

    void SimulationEntitiesMangerEditor::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesEditorService"));
    }

    void SimulationEntitiesMangerEditor::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SimulationEntitiesMangerEditor::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void SimulationEntitiesMangerEditor::Activate()
    {
        SimulationEntitiesManager::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void SimulationEntitiesMangerEditor::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        SimulationEntitiesManager::Deactivate();
    }

} // namespace SimulationInterfaces
