/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameEditorComponent.h"
#include "NamespaceComputation.h"
#include "ROS2FrameSystemBus.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameEditorComponentBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2
{
    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const ROS2FrameConfiguration ros2FrameConfiguration)
    {
        m_configuration = ros2FrameConfiguration;
    }

    void ROS2FrameEditorComponent::Init()
    {
    }

    void ROS2FrameEditorComponent::Activate()
    {
        ROS2FrameEditorComponentBus::Handler::BusConnect(GetEntityId());
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
        if (auto* frameSystemInterface = ROS2FrameSystemInterface::Get())
        {
            frameSystemInterface->RegisterFrame(GetEntityId());
        }
        UpdateNamespace();

        ROS2FrameComponentBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2FrameEditorComponent::Deactivate()
    {
        ROS2FrameComponentBus::Handler::BusDisconnect();

        if (auto* frameSystemInterface = ROS2FrameSystemInterface::Get())
        {
            frameSystemInterface->UnregisterFrame(GetEntityId());
        }
        AZ::EntityBus::Handler::BusDisconnect();
        ROS2FrameEditorComponentBus::Handler::BusDisconnect();
    }

    AZStd::string ROS2FrameEditorComponent::GetGlobalFrameID() const
    {
        const AZStd::string odometryFrame = GetGlobalFrameIDFromRegistry();
        const auto computedNamespace = ComputeNamespace(GetEntityId());
        return GetNamespacedName(computedNamespace, odometryFrame);
    }

    AZStd::string ROS2FrameEditorComponent::GetNamespacedFrameID() const
    {
        auto computedNamespace = ComputeNamespace(GetEntityId());
        return GetNamespacedName(computedNamespace, m_configuration.m_frameName);
    }

    AZStd::string ROS2FrameEditorComponent::GetNamespace() const
    {
        return ComputeNamespace(GetEntityId());
    }

    void ROS2FrameEditorComponent::UpdateNamespace()
    {
        m_effectiveNamespace = ComputeNamespace(GetEntityId());
        m_fullName = GetNamespacedName(m_effectiveNamespace, m_configuration.m_frameName);

        AzToolsFramework::PropertyEditorEntityChangeNotificationBus::Event(
            GetEntityId(), &AzToolsFramework::PropertyEditorEntityChangeNotificationBus::Events::OnEntityComponentPropertyChanged, GetId());
    }

    AZStd::string ROS2FrameEditorComponent::GetNamespacedJointName() const
    {
        auto computedNamespace = ComputeNamespace(GetEntityId());
        return GetNamespacedName(computedNamespace, m_configuration.m_jointName);
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
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
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
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Info")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->UIElement(AZ::Edit::UIHandlers::Label, "Effective namespace", "")
                    ->Attribute(AZ::Edit::Attributes::ValueText, &ROS2FrameEditorComponent::m_effectiveNamespace)
                    ->UIElement(AZ::Edit::UIHandlers::Label, "Full name", "")
                    ->Attribute(AZ::Edit::Attributes::ValueText, &ROS2FrameEditorComponent::m_fullName);
            }
        }
    }

    AZ::EntityId ROS2FrameEditorComponent::GetFrameParent() const
    {
        const auto ancestors = GetAllAncestorTransformBus(GetEntityId());
        return GetFirstEntityWithROS2FrameComponent(ancestors);
    }

    AZStd::set<AZ::EntityId> ROS2FrameEditorComponent::GetFrameDescendants() const
    {
        // get all descendants
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, GetEntityId(), &AZ::TransformBus::Events::GetAllDescendants);
        // filter only those with ROS2FrameComponent
        const auto ros2Children = GetEntitiesWithROS2FrameComponent(children);
        return AZStd::set<AZ::EntityId>(ros2Children.begin(), ros2Children.end());
    }

    AZ::Crc32 ROS2FrameEditorComponent::OnFrameConfigurationChange()
    {
        m_effectiveNamespace = ComputeNamespace(GetEntityId());
        m_fullName = GetNamespacedName(m_effectiveNamespace, m_configuration.m_frameName);
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

    void ROS2FrameEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<ROS2FrameComponent>(m_configuration);
    }

    ROS2FrameConfiguration ROS2FrameEditorComponent::GetConfiguration() const
    {
        return m_configuration;
    }

    void ROS2FrameEditorComponent::SetConfiguration(const ROS2FrameConfiguration& config)
    {
        AZ_Assert(GetEntity()->GetState() != AZ::Entity::State::Active, "API can be called only for disabled components");
        m_configuration = config;
    }

    AZStd::string ROS2FrameEditorComponent::GetJointName() const
    {
        return m_configuration.m_jointName;
    }

    AZStd::string ROS2FrameEditorComponent::GetFrameName() const
    {
        return m_configuration.m_frameName;
    }

} // namespace ROS2
