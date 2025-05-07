/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamedPoseEditorComponent.h"
#include "NamedPoseComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <LmbrCentral/Scripting/EditorTagComponentBus.h>
#include <SimulationInterfaces/NamedPose.h>

namespace SimulationInterfaces
{
    void NamedPoseEditorComponent::Activate()
    {
        EditorComponentBase::Activate();
        UpdateConfiguration();
        LmbrCentral::TagComponentNotificationsBus::Handler::BusConnect(GetEntityId());
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
    }

    void NamedPoseEditorComponent::Deactivate()
    {
        AZ::EntityBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        LmbrCentral::TagComponentNotificationsBus::Handler::BusDisconnect();
        EditorComponentBase::Deactivate();
    }

    void NamedPoseEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TagService")); // tag component
    }

    void NamedPoseEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        // Create Game component
        [[maybe_unused]] auto component = gameEntity->CreateComponent<NamedPoseComponent>(m_config);
        AZ_Assert(component, "NamedPoseEditorComponent:: failed to create runtime component");
    }

    void NamedPoseEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NamedPoseEditorComponent, AzToolsFramework::Components::EditorComponentBase>()->Version(0)->Field(
                "NamedPoseConfig", &NamedPoseEditorComponent::m_config);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<NamedPoseEditorComponent>("NamedPoseEditorComponent", "Component used to define names pose in simulation")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "NamedPoseEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Simulation Interfaces")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NamedPoseEditorComponent::m_config, "NamedPoseConfig", "");
            }
        }
    }

    void NamedPoseEditorComponent::OnTagAdded(const LmbrCentral::Tag&)
    {
        LmbrCentral::EditorTagComponentRequestBus::EventResult(
            m_config.m_tags, GetEntityId(), &LmbrCentral::EditorTagComponentRequests::GetTags);
    }
    void NamedPoseEditorComponent::OnTagRemoved(const LmbrCentral::Tag&)
    {
        LmbrCentral::EditorTagComponentRequestBus::EventResult(
            m_config.m_tags, GetEntityId(), &LmbrCentral::EditorTagComponentRequests::GetTags);
    }

    void NamedPoseEditorComponent::OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world)
    {
        m_config.m_pose = world;
    }

    void NamedPoseEditorComponent::OnEntityNameChanged(const AZStd::string& name)
    {
        m_config.m_name = name;
    }

    void NamedPoseEditorComponent::UpdateConfiguration()
    {
        // gather all informations to create entity
        // name of named pose is derived from the entity name
        AZ::ComponentApplicationBus::BroadcastResult(m_config.m_name, &AZ::ComponentApplicationRequests::GetEntityName, GetEntityId());
        // pose is derived from transform component
        AZ::TransformBus::EventResult(m_config.m_pose, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        // tags are taken from editor tag component
        LmbrCentral::EditorTagComponentRequestBus::EventResult(
            m_config.m_tags, GetEntityId(), &LmbrCentral::EditorTagComponentRequests::GetTags);
    }
} // namespace SimulationInterfaces
