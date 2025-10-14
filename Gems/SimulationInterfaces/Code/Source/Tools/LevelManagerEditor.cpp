/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LevelManagerEditor.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    AZ_COMPONENT_IMPL(LevelManagerEditor, "SimulationMangerEditor", LevelManagerEditorTypeId, BaseSystemComponent);

    void LevelManagerEditor::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LevelManagerEditor, LevelManager>()->Version(0);
        }
    }

    LevelManagerEditor::LevelManagerEditor() = default;

    LevelManagerEditor::~LevelManagerEditor() = default;

    void LevelManagerEditor::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("LevelManagerEditorService"));
    }

    void LevelManagerEditor::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("LevelManagerEditorService"));
    }

    void LevelManagerEditor::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }

    void LevelManagerEditor::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregatorEditorService"));
    }
    void LevelManagerEditor::Init()
    {
        BaseSystemComponent::Init();
    }

    void LevelManagerEditor::Activate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusConnect();
    }

    void LevelManagerEditor::Deactivate()
    {
        AzToolsFramework::EditorEntityContextNotificationBus::Handler::BusDisconnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
    }

    void LevelManagerEditor::OnStartPlayInEditorBegin()
    {
        BaseSystemComponent::Activate();
    }
    void LevelManagerEditor::OnStopPlayInEditorBegin()
    {
        BaseSystemComponent::Deactivate();
    }

} // namespace SimulationInterfaces
