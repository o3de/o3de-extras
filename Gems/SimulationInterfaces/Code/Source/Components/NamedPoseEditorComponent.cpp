/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamedPoseEditorComponent.h"
#include "Components/NamedPoseComponent.h"
#include "SimulationInterfaces/NamedPose.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <LmbrCentral/Scripting/EditorTagComponentBus.h>

namespace SimulationInterfaces
{

    void NamedPoseEditorComponent::Activate()
    {
        EditorComponentBase::Activate();
        UpdateConfiguration();
    }

    void NamedPoseEditorComponent::Deactivate()
    {
        EditorComponentBase::Deactivate();
    }

    void NamedPoseEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TagService")); // tag component
    }

    void NamedPoseEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        // Create Game component
        UpdateConfiguration();
        gameEntity->CreateComponent<NamedPoseComponent>(m_config);
    }

    void NamedPoseEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            if (!context->IsTypeReflected(azrtti_typeid<NamedPose>()))
            {
                NamedPose::Reflect(context);
            }
            serialize->Class<NamedPoseEditorComponent, EditorComponentBase>()->Version(0)->Field(
                "NamedPoseConfig", &NamedPoseEditorComponent::m_config);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<NamedPoseEditorComponent>("Named Pose Component", "Component used to define names pose in simulation")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "NamedPoseEditorComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Simulation Interfaces")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NamedPoseEditorComponent::m_config, "Named Pose Config", "");
            }
        }
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
