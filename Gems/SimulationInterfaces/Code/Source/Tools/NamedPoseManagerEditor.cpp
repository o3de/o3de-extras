/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamedPoseManagerEditor.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <Clients/NamedPosesManager.h>

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    AZ_COMPONENT_IMPL(NamedPoseManagerEditor, "NamedPoseManagerEditor", NamedPoseManagerEditorTypeId, BaseSystemComponent);

    void NamedPoseManagerEditor::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<NamedPoseManagerEditor, NamedPoseManager>()->Version(0);
        }
    }

    NamedPoseManagerEditor::NamedPoseManagerEditor() = default;

    NamedPoseManagerEditor::~NamedPoseManagerEditor() = default;

    void NamedPoseManagerEditor::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("NamedPoseManagerEditorService"));
    }

    void NamedPoseManagerEditor::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("NamedPoseManagerEditorService"));
    }

    void NamedPoseManagerEditor::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }

    void NamedPoseManagerEditor::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }

    void NamedPoseManagerEditor::Activate()
    {
        BaseSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void NamedPoseManagerEditor::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        BaseSystemComponent::Deactivate();
    }

} // namespace SimulationInterfaces
