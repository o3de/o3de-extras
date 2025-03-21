/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include "SimulationInterfacesEditorSystemComponent.h"

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    AZ_COMPONENT_IMPL(SimulationInterfacesEditorSystemComponent, "SimulationInterfacesEditorSystemComponent",
        SimulationInterfacesEditorSystemComponentTypeId, BaseSystemComponent);

    void SimulationInterfacesEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationInterfacesEditorSystemComponent, SimulationEntitiesManager>()
                ->Version(0);
        }
    }

    SimulationInterfacesEditorSystemComponent::SimulationInterfacesEditorSystemComponent() = default;

    SimulationInterfacesEditorSystemComponent::~SimulationInterfacesEditorSystemComponent() = default;

    void SimulationInterfacesEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SimulationInterfacesEditorService"));
    }

    void SimulationInterfacesEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesEditorService"));
    }

    void SimulationInterfacesEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SimulationInterfacesEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void SimulationInterfacesEditorSystemComponent::Activate()
    {
        SimulationEntitiesManager::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void SimulationInterfacesEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        SimulationEntitiesManager::Deactivate();
    }

} // namespace SimulationInterfaces
