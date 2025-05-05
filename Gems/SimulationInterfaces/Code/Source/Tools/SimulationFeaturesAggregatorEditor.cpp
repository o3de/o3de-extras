/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationFeaturesAggregatorEditor.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    AZ_COMPONENT_IMPL(
        SimulationFeaturesAggregatorEditor, "SimulationMangerEditor", SimulationFeaturesAggregatorEditorTypeId, BaseSystemComponent);

    void SimulationFeaturesAggregatorEditor::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationFeaturesAggregatorEditor, SimulationFeaturesAggregator>()->Version(0);
        }
    }

    SimulationFeaturesAggregatorEditor::SimulationFeaturesAggregatorEditor() = default;

    SimulationFeaturesAggregatorEditor::~SimulationFeaturesAggregatorEditor() = default;

    void SimulationFeaturesAggregatorEditor::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }

    void SimulationFeaturesAggregatorEditor::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }

    void SimulationFeaturesAggregatorEditor::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void SimulationFeaturesAggregatorEditor::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }
    void SimulationFeaturesAggregatorEditor::Init()
    {
        BaseSystemComponent::Init();
    }

    void SimulationFeaturesAggregatorEditor::Activate()
    {
        BaseSystemComponent::Activate();
    }

    void SimulationFeaturesAggregatorEditor::Deactivate()
    {
        BaseSystemComponent::Deactivate();
    }

} // namespace SimulationInterfaces
