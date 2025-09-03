/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamespaceComputation.h"
#include "ROS2FrameSystemBus.h"
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/Json/RegistrationContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <rapidjson/stringbuffer.h>

namespace ROS2
{

    namespace Internal
    {
        bool HasComponentOfType(const AZ::Entity* entity, const AZ::Uuid typeId)
        {
            auto components = AZ::EntityUtils::FindDerivedComponents(entity, typeId);
            return !components.empty();
        }

        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity)
        {
            if (!entity)
            {
                AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
                return nullptr;
            }

            auto* interface = entity->FindComponent<AzFramework::TransformComponent>();
            return interface;
        }

        const ROS2FrameComponent* GetFirstROS2FrameAncestor(const AZ::Entity* entity)
        {
            auto* entityTransformInterface = GetEntityTransformInterface(entity);
            if (!entityTransformInterface)
            {
                AZ_Error("GetFirstROS2FrameAncestor", false, "Invalid transform interface!");
                return nullptr;
            }

            AZ::EntityId parentEntityId = entityTransformInterface->GetParentId();
            if (!parentEntityId.IsValid())
            { // We have reached the top level, there is no parent entity so there can be no parent ROS2Frame
                return nullptr;
            }
            AZ::Entity* parentEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, parentEntityId);
            AZ_Assert(parentEntity, "No parent entity id : %s", parentEntityId.ToString().c_str());

            auto* component = parentEntity->FindComponent<ROS2FrameComponent>();
            if (component == nullptr)
            { // Parent entity has no ROS2Frame, but there can still be a ROS2Frame in its ancestors
                return GetFirstROS2FrameAncestor(parentEntity);
            }

            // Found the component!
            return component;
        }

        bool IsDynamicHeuristic(bool isTopLevel, const AZ::Entity* entity)
        {
            if (isTopLevel)
            {
                return true;
            }
            const bool hasJoints =
                Internal::HasComponentOfType(entity, AZ::Uuid("{B01FD1D2-1D91-438D-874A-BF5EB7E919A8}")); // PhysX::JointComponent;
            const bool hasFixedJoints =
                Internal::HasComponentOfType(entity, AZ::Uuid("{02E6C633-8F44-4CEE-AE94-DCB06DE36422}")); // PhysX::FixedJointComponent
            const bool hasArticulations =
                Internal::HasComponentOfType(entity, AZ::Uuid("{48751E98-B35F-4A2F-A908-D9CDD5230264}")); // PhysX::ArticulationComponent
            return (hasJoints && !hasFixedJoints) || hasArticulations;
        }

    } // namespace Internal

    void ROS2FrameComponent::Init()
    {
    }

    void ROS2FrameComponent::Activate()
    {
        // reset cache
        m_parentFrame.reset();
        m_sourceFrame.reset();
        ComputeNamespaceAndFrameName();
        if (!m_disabled)
        {
            AZ::TickBus::Handler::BusConnect();
        }

        ROS2FrameComponentBus::Handler::BusConnect(GetEntityId());

        if (auto* frameSystemInterface = ROS2FrameSystemInterface::Get())
        {
            frameSystemInterface->RegisterFrame(GetEntityId());
        }
    }

    void ROS2FrameComponent::Deactivate()
    {
        if (auto* frameSystemInterface = ROS2FrameSystemInterface::Get())
        {
            frameSystemInterface->UnregisterFrame(GetEntityId());
        }

        ROS2FrameComponentBus::Handler::BusDisconnect(GetEntityId());

        AZ::TickBus::Handler::BusDisconnect();
        m_parentFrame.reset();
        m_sourceFrame.reset();
        m_ros2Transform.reset();
    }

    void ROS2FrameComponent::ComputeNamespaceAndFrameName()
    {
        m_computedNamespace = ComputeNamespace(GetEntityId());
        m_computedFrameName = GetNamespacedName(m_computedNamespace, m_configuration.m_frameName);
        m_computedJointName = GetNamespacedName(m_computedNamespace, m_configuration.m_jointName);
    }

    void ROS2FrameComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_sourceFrame == AZStd::nullopt)
        {
            // cache parent frame
            const auto* ros2FrameComponent = Internal::GetFirstROS2FrameAncestor(GetEntity());
            if (ros2FrameComponent != nullptr)
            {
                m_parentFrame = ros2FrameComponent->GetEntityId();
                m_sourceFrame = ros2FrameComponent->GetNamespacedFrameID();
            }
            else
            {
                m_parentFrame = AZStd::nullopt;
                AZStd::string odometryFrame = GetGlobalFrameID();
                if (!odometryFrame.empty())
                {
                    m_sourceFrame = odometryFrame;
                }
            }
        }

        // if we don't have to send transforms stop handler
        if (!m_configuration.m_publishTransform)
        {
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }

        if (m_ros2Transform == nullptr && m_sourceFrame.has_value())
        {
            const bool isTopLevel = !m_parentFrame.has_value();
            const bool dynamic = m_configuration.m_forceDynamic || Internal::IsDynamicHeuristic(isTopLevel, GetEntity());
            AZ_Printf(
                "m_ros2Transform",
                "publishing transform from %s to %s, type %s",
                m_sourceFrame->c_str(),
                GetNamespacedFrameID().c_str(),
                dynamic ? "dynamic" : "static");

            m_ros2Transform = AZStd::make_unique<ROS2Transform>(*m_sourceFrame, GetNamespacedFrameID(), dynamic);
            m_ros2Transform->Publish(GetFrameTransform());
            if (!dynamic)
            {
                // static transform published, no need to keep ticking
                AZ::TickBus::Handler::BusDisconnect();
                return;
            }
        }
        if (m_ros2Transform)
        {
            m_ros2Transform->Publish(GetFrameTransform());
        }
    }

    const ROS2FrameComponent* ROS2FrameComponent::GetParentROS2FrameComponent() const
    {
        if (m_parentFrame.has_value())
        {
            AZ::Entity* parentEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, *m_parentFrame);
            AZ_Assert(parentEntity, "No parent entity id : %s", m_parentFrame->ToString().c_str());

            return parentEntity->FindComponent<ROS2FrameComponent>();
        }
        return nullptr;
    }

    AZ::Transform ROS2FrameComponent::GetFrameTransform() const
    {
        auto* transformInterface = Internal::GetEntityTransformInterface(GetEntity());
        if (const auto* parentFrame = GetParentROS2FrameComponent(); parentFrame != nullptr)
        {
            auto* ancestorTransformInterface = Internal::GetEntityTransformInterface(parentFrame->GetEntity());
            AZ_Assert(ancestorTransformInterface, "No transform interface for an entity with a ROS2Frame component, which requires it!");

            const auto worldFromAncestor = ancestorTransformInterface->GetWorldTM();
            const auto worldFromThis = transformInterface->GetWorldTM();
            const auto ancestorFromWorld = worldFromAncestor.GetInverse();
            return ancestorFromWorld * worldFromThis;
        }
        return transformInterface->GetWorldTM();
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2FrameConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, AZ::Component>()->Version(1)->Field(
                "ROS2FrameConfiguration", &ROS2FrameComponent::m_configuration);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameComponent>(
                      "ROS2 Frame Game Component (outdated)",
                      "This is a game version of the ROS2 Frame component. This is outdated and was updated to the new "
                      "ROS2FrameEditorComponent. If you see this component a manual conversion is required.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Icons/Components/ROS2Frame.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Icons/Components/Viewport/ROS2Frame.svg")
                    ->Attribute(AZ::Edit::Attributes::HelpPageURL, "https://o3de.org/docs/user-guide/components/reference/ros2-frame/");
            }
        }
        if (auto bh = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            bh->EBus<ROS2FrameComponentBus>("ROS2FrameComponentBus")
                ->Event("GetNamespace", &ROS2FrameComponentBus::Events::GetNamespace)
                ->Event("GetNamespacedFrameID", &ROS2FrameComponentBus::Events::GetNamespacedFrameID)
                ->Event("GetNamespacedJointName", &ROS2FrameComponentBus::Events::GetNamespacedJointName)
                ->Event("GetJointName", &ROS2FrameComponentBus::Events::GetJointName)
                ->Event("GetFrameName", &ROS2FrameComponentBus::Events::GetFrameName)
                ->Event("GetGlobalFrameID", &ROS2FrameComponentBus::Events::GetGlobalFrameID);
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
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    ROS2FrameComponent::ROS2FrameComponent(){};

    ROS2FrameComponent::ROS2FrameComponent(const ROS2FrameConfiguration& ros2FrameConfiguration)
        : m_configuration(ros2FrameConfiguration)
    {
    }

    //! Disable publishing the transform. It allows to tinker with the transform without spamming /tf_static
    //! you can disable it inactive state, and then enable it again.
    void ROS2FrameComponent::DisablePublishingTransform()
    {
        m_disabled = true;
        m_parentFrame.reset();
        m_ros2Transform.reset();
        m_sourceFrame.reset();
        AZ::TickBus::Handler::BusDisconnect();
    }

    //! Enable publishing the transform. It will compute the namespace and frame name again.
    //! Note that other won't be notified of the change.
    void ROS2FrameComponent::EnablePublishingTransform()
    {
        ComputeNamespaceAndFrameName();
        m_disabled = false;
        AZ::TickBus::Handler::BusConnect();
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        return m_computedNamespace;
    }

    AZStd::string ROS2FrameComponent::GetNamespacedFrameID() const
    {
        return m_computedFrameName;
    }

    AZStd::string ROS2FrameComponent::GetNamespacedJointName() const
    {
        return m_computedJointName;
    }

    AZStd::string ROS2FrameComponent::GetJointName() const
    {
        return m_configuration.m_jointName;
    }

    AZStd::string ROS2FrameComponent::GetFrameName() const
    {
        return m_configuration.m_frameName;
    }

    ROS2FrameConfiguration ROS2FrameComponent::GetConfiguration() const
    {
        return m_configuration;
    }

    void ROS2FrameComponent::SetConfiguration(const ROS2FrameConfiguration& config)
    {
        if (m_entity)
        {
            AZ_Assert(GetEntity()->GetState() != AZ::Entity::State::Active, "API can be called only for disabled components");
        }
        if (!config.m_namespaceConfiguration.m_customNamespace.empty())
        {
            AZ_Warning(
                "ROS2FrameComponent",
                config.m_namespaceConfiguration.m_namespaceStrategy == NamespaceConfiguration::NamespaceStrategy::Custom,
                "Custom namespace is set but the namespace strategy is not set to Custom. The custom namespace will be ignored.");
        }
        m_configuration = config;
    }

    AZStd::string ROS2FrameComponent::GetGlobalFrameID() const
    {
        AZStd::string odometryFrame;
        auto* registry = AZ::SettingsRegistry::Get();
        AZ_Error("ROS2FrameComponent", registry, "No settings registry found, using default odometry frame name");
        if (registry)
        {
            if (!registry->Get(odometryFrame, DefaultGlobalFrameNameConfigurationKey))
            {
                odometryFrame = DefaultGlobalFrameName;
            }
        }
        if (odometryFrame.empty())
        {
            return "";
        }
        return GetNamespacedName(m_computedNamespace, odometryFrame);
    }

} // namespace ROS2
