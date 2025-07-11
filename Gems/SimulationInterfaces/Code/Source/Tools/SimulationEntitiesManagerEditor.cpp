/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationEntitiesManagerEditor.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    AZ_COMPONENT_IMPL(
        SimulationEntitiesManagerEditor, "SimulationEntitiesManagerEditor", SimulationEntitiesManagerEditorTypeId, BaseSystemComponent);

    void SimulationEntitiesManagerEditor::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationEntitiesManagerEditor, SimulationEntitiesManager>()->Version(0);
        }
    }

    SimulationEntitiesManagerEditor::SimulationEntitiesManagerEditor() = default;

    SimulationEntitiesManagerEditor::~SimulationEntitiesManagerEditor() = default;

    void SimulationEntitiesManagerEditor::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SimulationInterfacesEditorService"));
    }

    void SimulationEntitiesManagerEditor::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesEditorService"));
    }

    void SimulationEntitiesManagerEditor::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }

    void SimulationEntitiesManagerEditor::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }

    void SimulationEntitiesManagerEditor::Activate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void SimulationEntitiesManagerEditor::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
    }

    void SimulationEntitiesManagerEditor::OnStartPlayInEditorBegin()
    {
        SimulationEntitiesManager::Activate();
    }

    void SimulationEntitiesManagerEditor::OnStopPlayInEditorBegin()
    {
        SimulationEntitiesManager::Deactivate();
    }

} // namespace SimulationInterfaces
