/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "API/ToolsApplicationAPI.h"
#include "AzCore/Component/EntityBus.h"
#include "AzCore/Component/TransformBus.h"
#include "ROS2/Frame/ROS2FrameBus.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameController.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace Internal
    {
        void PopulateNonFramePredecessors(AZ::EntityId entity, AZStd::set<AZ::EntityId>& nonFramePredecessors)
        {
            AZ::EntityId parentEntityId;
            AZ::TransformBus::EventResult(parentEntityId, entity, &AZ::TransformBus::Events::GetParentId);
            if (!parentEntityId.IsValid())
            { // We have reached the top level, there is no parent entity so there can be no parent ROS2Frame
                return;
            }

            bool hasFrame = false;
            ROS2FrameComponentBus::EventResult(hasFrame, parentEntityId, &ROS2FrameComponentBus::Events::IsFrame);

            if (hasFrame)
            { // Found first frame predecessor
                return;
            }

            nonFramePredecessors.insert(parentEntityId);
            PopulateNonFramePredecessors(parentEntityId, nonFramePredecessors);
        }

        AZStd::set<AZ::EntityId> GetNonFramePredecessors(AZ::EntityId entity)
        {
            AZStd::set<AZ::EntityId> nonFramePredecessors;
            PopulateNonFramePredecessors(entity, nonFramePredecessors);
            return nonFramePredecessors;
        }

    } // namespace Internal

    void ROS2FrameEditorComponent::Activate()
    {
        ROS2FrameEditorComponentBase::Activate();

        m_nonFramePredecessors = Internal::GetNonFramePredecessors(GetEntityId());

        if (m_controller.IsTopLevel())
        {
            m_parentFrameEntity.SetInvalid();
        }
        else
        {
            m_parentFrameEntity = m_controller.GetParentROS2FrameEntityId();
        }

        m_controller.PopulateNamespace(m_controller.IsTopLevel(), GetEntity()->GetName());
        RefreshEffectiveNamespace();

        ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnActivate, GetEntityId(), m_parentFrameEntity);

        ROS2FrameNotificationBus::Handler::BusConnect(GetEntityId());
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
        AzToolsFramework::ToolsApplicationNotificationBus::Handler::BusConnect();
    }

    void ROS2FrameEditorComponent::Deactivate()
    {
        AzToolsFramework::ToolsApplicationNotificationBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
        ROS2FrameNotificationBus::Handler::BusDisconnect();

        ROS2FrameEditorComponentBase::Deactivate();

        ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnDeactivate, GetEntityId(), m_parentFrameEntity);
    }

    void ROS2FrameEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2FrameEditorComponentBase::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameEditorComponent, ROS2FrameEditorComponentBase>()->Version(1)->Field(
                "Effective namespace", &ROS2FrameEditorComponent::m_effectiveNamespace);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameEditorComponent>("ROS2 Frame", "[ROS2 Frame component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameEditorComponent::m_effectiveNamespace, "Effective namespace", "")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, true);
            }
        }
    }

    void ROS2FrameEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    ROS2FrameEditorComponent::ROS2FrameEditorComponent() = default;

    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const AZStd::string& frameId)
    {
        m_controller.SetFrameID(frameId);
    }

    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const ROS2FrameConfiguration& config)
    {
        SetConfiguration(config);
    }

    bool ROS2FrameEditorComponent::ShouldActivateController() const
    {
        return true;
    }

    void ROS2FrameEditorComponent::OnActivate(AZ::EntityId entity, AZ::EntityId parentEntity)
    {
        if (m_parentFrameEntity == parentEntity && m_controller.GetParentROS2FrameEntityId() == entity)
        {
            m_parentFrameEntity = entity;
            m_controller.PopulateNamespace(false, GetEntity()->GetName());
            RefreshEffectiveNamespace();
            ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnReconfigure, GetEntityId());
        }
    }

    void ROS2FrameEditorComponent::OnDeactivate(AZ::EntityId entity, AZ::EntityId parentEntity)
    {
        if (m_parentFrameEntity == entity)
        {
            m_parentFrameEntity = parentEntity;
            m_controller.PopulateNamespace(parentEntity.IsValid(), GetEntity()->GetName());
            RefreshEffectiveNamespace();
            ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnReconfigure, GetEntityId());
        }
    }

    void ROS2FrameEditorComponent::OnConfigurationChange()
    {
        m_controller.PopulateNamespace(m_controller.IsTopLevel(), GetEntity()->GetName());
        RefreshEffectiveNamespace();

        ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnReconfigure, GetEntityId());
    }

    void ROS2FrameEditorComponent::OnReconfigure(AZ::EntityId entity)
    {
        if (m_parentFrameEntity == entity)
        {
            m_controller.PopulateNamespace(false, GetEntity()->GetName());
            RefreshEffectiveNamespace();

            ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnReconfigure, GetEntityId());
        }
    }

    void ROS2FrameEditorComponent::EntityParentChanged(AZ::EntityId entityId, AZ::EntityId newParentId, AZ::EntityId oldParentId)
    {
        if (GetEntityId() == entityId || m_nonFramePredecessors.contains(entityId))
        {
            bool isTopLevel = m_controller.IsTopLevel();
            m_controller.PopulateNamespace(isTopLevel, GetEntity()->GetName());
            RefreshEffectiveNamespace();

            m_nonFramePredecessors = Internal::GetNonFramePredecessors(GetEntityId());

            ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnDeactivate, GetEntityId(), m_parentFrameEntity);

            if (isTopLevel)
            {
                m_parentFrameEntity.SetInvalid();
            }
            else
            {
                m_parentFrameEntity = m_controller.GetParentROS2FrameEntityId();
            }

            ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnActivate, GetEntityId(), m_parentFrameEntity);
        }
    }

    void ROS2FrameEditorComponent::OnEntityNameChanged(const AZStd::string& name)
    {
        (void)name;
        m_controller.PopulateNamespace(false, GetEntity()->GetName());
        RefreshEffectiveNamespace();

        ROS2FrameNotificationBus::Broadcast(&ROS2FrameNotificationBus::Events::OnReconfigure, GetEntityId());
    }

    void ROS2FrameEditorComponent::RefreshEffectiveNamespace()
    {
        m_effectiveNamespace = m_controller.GetNamespace();
        if (const AZ::Component* component = GetEntity()->FindComponent<ROS2FrameEditorComponent>())
        {
            AzToolsFramework::PropertyEditorEntityChangeNotificationBus::Event(
                GetEntityId(),
                &AzToolsFramework::PropertyEditorEntityChangeNotifications::OnEntityComponentPropertyChanged,
                component->GetId());
        }
    }

} // namespace ROS2
