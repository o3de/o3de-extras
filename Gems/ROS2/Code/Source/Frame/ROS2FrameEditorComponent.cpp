/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameSystemBus.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <ROS2/Frame/ROS2FrameBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const ROS2FrameConfiguration ros2FrameConfiguration)
    {
        m_configuration = ros2FrameConfiguration;
    }

    void ROS2FrameEditorComponent::Activate()
    {
        ROS2FrameComponentBus::Handler::BusConnect(GetEntityId());
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
        ROS2FrameSystemInterface::Get()->RegisterFrame(GetEntityId());
    }

    void ROS2FrameEditorComponent::Deactivate()
    {
        ROS2FrameSystemInterface::Get()->UnregisterFrame(GetEntityId());
        AZ::EntityBus::Handler::BusDisconnect();
        ROS2FrameComponentBus::Handler::BusDisconnect();
    }

    AZStd::string ROS2FrameEditorComponent::GetGlobalFrameName() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), AZStd::string("odom"));
    }

    bool ROS2FrameEditorComponent::IsTopLevel() const
    {
        return ROS2FrameSystemInterface::Get()->IsTopLevel(GetEntityId());
    }

    AZStd::string ROS2FrameEditorComponent::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), m_configuration.m_frameName);
    }

    void ROS2FrameEditorComponent::SetFrameID(const AZStd::string& frameId)
    {
        m_configuration.m_frameName = frameId;
    }

    AZStd::string ROS2FrameEditorComponent::GetNamespace() const
    {
        return m_configuration.m_namespaceConfiguration.GetNamespace();
    }

    void ROS2FrameEditorComponent::UpdateNamespace(const AZStd::string& parentNamespace)
    {
        m_configuration.m_namespaceConfiguration.SetParentNamespace(parentNamespace);
        m_configuration.m_namespaceConfiguration.PopulateNamespace(IsTopLevel(), GetEntity()->GetName());
        m_configuration.SetEffectiveNamespace(GetNamespace());
        AzToolsFramework::PropertyEditorEntityChangeNotificationBus::Event(
            GetEntityId(),
            &AzToolsFramework::PropertyEditorEntityChangeNotificationBus::Events::OnEntityComponentPropertyChanged,
            GetEntity()->FindComponent<ROS2FrameEditorComponent>()->GetId());

        ROS2FrameComponentNotificationBus::Event(GetEntityId(), &ROS2FrameComponentNotificationBus::Events::OnConfigurationChange);
    }

    void ROS2FrameEditorComponent::UpdateNamespaceConfiguration(
        const AZStd::string& ros2Namespace, const NamespaceConfiguration::NamespaceStrategy& strategy)
    {
        m_configuration.m_namespaceConfiguration.SetNamespace(ros2Namespace, strategy);
    }

    AZ::Name ROS2FrameEditorComponent::GetJointName() const
    {
        return AZ::Name(ROS2Names::GetNamespacedName(GetNamespace(), m_configuration.m_jointName).c_str());
    }

    void ROS2FrameEditorComponent::SetJointName(const AZStd::string& jointName)
    {
        m_configuration.m_jointName = jointName;
    }

    void ROS2FrameEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameEditorComponent, AzToolsFramework::Components::EditorComponentBase>()->Version(1)->Field(
                "ROS2FrameConfiguration", &ROS2FrameEditorComponent::m_configuration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameEditorComponent>("ROS2 Frame", "[ROS2 Frame component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2Frame.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2Frame.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameEditorComponent::m_configuration,
                        "ROS 2 Frame Configuration",
                        "Configuration of ROS 2 reference frame")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ROS2FrameEditorComponent::OnFrameConfigurationChange)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues);
            }
        }
    }

    AZ::EntityId ROS2FrameEditorComponent::GetFrameParent() const
    {
        return ROS2FrameSystemInterface::Get()->GetParentEntityId(GetEntityId());
    }

    AZStd::set<AZ::EntityId> ROS2FrameEditorComponent::GetFrameChildren() const
    {
        return ROS2FrameSystemInterface::Get()->GetChildrenEntityId(GetEntityId());
    }

    AZ::Crc32 ROS2FrameEditorComponent::OnFrameConfigurationChange()
    {
        ROS2FrameSystemInterface::Get()->NotifyChange(GetEntityId());
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    void ROS2FrameEditorComponent::OnEntityNameChanged(const AZStd::string& name)
    {
        OnFrameConfigurationChange();
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

    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const AZStd::string& frameId)
    {
        SetFrameID(frameId);
    }

    void ROS2FrameEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<ROS2FrameComponent>(m_configuration);
    }

} // namespace ROS2
