/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "Utilities/ROS2Names.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>

namespace ROS2
{
    void ROS2FrameComponent::Activate()
    {
        m_namespaceConfiguration.PopulateNamespace(IsTopLevel(), GetEntity()->GetName());

        if (m_publishTransform)
        {
            AZ_TracePrintf(
                "ROS2FrameComponent",
                "Setting up %s transfrom between parent %s and child %s to be published %s",
                IsDynamic() ? "dynamic" : "static",
                GetParentFrameID().data(),
                GetFrameID().data(),
                IsDynamic() ? "continuously to /tf" : "once to /tf_static");

            m_ros2Transform = AZStd::make_unique<ROS2Transform>(GetParentFrameID(), GetFrameID(), IsDynamic());
            if (IsDynamic())
            {
                AZ::TickBus::Handler::BusConnect();
            }
            else
            {
                m_ros2Transform->Publish(GetFrameTransform());
            }
        }
    }

    void ROS2FrameComponent::Deactivate()
    {
        if (m_publishTransform)
        {
            if (IsDynamic())
            {
                AZ::TickBus::Handler::BusDisconnect();
            }
            m_ros2Transform.reset();
        }
    }

    void ROS2FrameComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_ros2Transform->Publish(GetFrameTransform());
    }

    const char* ROS2FrameComponent::GetGlobalFrameName()
    {
        // TODO - parametrize this (typically: "odom", "world" and sometimes "map")
        return "odom";
    }

    bool ROS2FrameComponent::IsTopLevel() const
    {
        return GetGlobalFrameName() == GetParentFrameID();
    }

    bool ROS2FrameComponent::IsDynamic() const
    { // TODO - determine by joint type
        return IsTopLevel();
    }

    const ROS2FrameComponent* ROS2FrameComponent::GetParentROS2FrameComponent() const
    {
        auto* transformInterface = GetEntityTransformInterface();
        if (!transformInterface)
        {
            AZ_Error("GetParentROS2FrameComponent", false, "No transform interface, but it is required!");
            return nullptr;
        }

        if (AZ::EntityId parentEntityId = GetEntityTransformInterface()->GetParentId(); parentEntityId.IsValid())
        {
            const AZ::Entity* parentEntity = AzToolsFramework::GetEntityById(parentEntityId);
            auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(parentEntity);
            return component;
        }
        return nullptr;
    }

    const AZ::Transform& ROS2FrameComponent::GetFrameTransform() const
    {
        auto transformInterface = GetEntityTransformInterface();
        if (GetParentROS2FrameComponent() != nullptr)
        {
            return transformInterface->GetLocalTM();
        }
        return transformInterface->GetWorldTM();
    }

    AZ::TransformInterface* ROS2FrameComponent::GetEntityTransformInterface() const
    {
        // TODO - instead, use EditorFrameComponent to handle Editor-context queries and here only use the "Game" version
        auto* interface = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        if (interface)
        {
            return interface;
        }
        return GetEntity()->FindComponent<AzToolsFramework::Components::TransformComponent>();
    }

    AZStd::string ROS2FrameComponent::GetParentFrameID() const
    {
        if (auto parentFrame = GetParentROS2FrameComponent(); parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }
        return GetGlobalFrameName();
    }

    AZStd::string ROS2FrameComponent::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), m_frameName);
    }

    void ROS2FrameComponent::SetFrameID(const AZStd::string& frameId)
    {
        m_frameName = frameId;
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        auto parentFrame = GetParentROS2FrameComponent();
        AZStd::string parentNamespace("");
        if (parentFrame != nullptr)
        {
            parentNamespace = parentFrame->GetNamespace();
        }
        return m_namespaceConfiguration.GetNamespace(parentNamespace);
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        NamespaceConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, AZ::Component>()
                ->Version(1)
                ->Field("Namespace Configuration", &ROS2FrameComponent::m_namespaceConfiguration)
                ->Field("Frame Name", &ROS2FrameComponent::m_frameName)
                ->Field("Publish Transform", &ROS2FrameComponent::m_publishTransform);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameComponent>("ROS2 Frame", "[ROS2 Frame component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameComponent::m_namespaceConfiguration,
                        "Namespace Configuration",
                        "Namespace Configuration")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameComponent::m_frameName, "Frame Name", "Frame Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2FrameComponent::m_publishTransform, "Publish Transform", "Publish Transform");
            }
        }
    }

    void ROS2FrameComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService"));
    }

    ROS2FrameComponent::ROS2FrameComponent() = default;

    ROS2FrameComponent::ROS2FrameComponent(AZStd::string frameId)
        : m_frameName(AZStd::move(frameId))
    {
    }
} // namespace ROS2
