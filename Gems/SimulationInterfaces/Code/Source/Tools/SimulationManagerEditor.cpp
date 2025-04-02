/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationManagerEditor.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    AZ_COMPONENT_IMPL(SimulationManagerEditor, "SimulationMangerEditor", SimulationManagerEditorTypeId, BaseSystemComponent);

    void SimulationManagerEditor::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationManagerEditor, SimulationManager>()->Version(0);
        }
    }

    SimulationManagerEditor::SimulationManagerEditor() = default;

    SimulationManagerEditor::~SimulationManagerEditor() = default;

    void SimulationManagerEditor::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SimulationManagerEditorService"));
    }

    void SimulationManagerEditor::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SimulationManagerEditorService"));
    }

    void SimulationManagerEditor::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SimulationManagerEditor::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }
    void SimulationManagerEditor::Init()
    {
        BaseSystemComponent::Init();
    }

    void SimulationManagerEditor::Activate()
    {
        BaseSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void SimulationManagerEditor::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        BaseSystemComponent::Deactivate();
    }

} // namespace SimulationInterfaces
