/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2FrameRegistrationBus.h>
#include <ROS2/ROS2NamesBus.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/TransformComponent.h>

namespace ROS2
{
    namespace Internal
    {
        // PhysX component UUIDs used for dynamic detection
        // These are hardcoded to avoid linking dependencies
        constexpr AZ::Uuid PhysXJointComponentUuid = AZ::Uuid("{B01FD1D2-1D91-438D-874A-BF5EB7E919A8}");
        constexpr AZ::Uuid PhysXFixedJointComponentUuid = AZ::Uuid("{02E6C633-8F44-4CEE-AE94-DCB06DE36422}");
        constexpr AZ::Uuid PhysXArticulationComponentUuid = AZ::Uuid("{48751E98-B35F-4A2F-A908-D9CDD5230264}");

        bool HasComponentOfType(const AZ::Entity* entity, const AZ::Uuid& typeId)
        {
            if (!entity)
            {
                return false;
            }

            const auto components = AZ::EntityUtils::FindDerivedComponents(entity, typeId);
            return !components.empty();
        }

        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity)
        {
            if (!entity)
            {
                AZ_Error("ROS2FrameComponent", false, "Cannot get transform interface for null entity");
                return nullptr;
            }

            return entity->FindComponent<AzFramework::TransformComponent>();
        }

        const ROS2FrameComponent* GetFirstROS2FrameAncestor(const AZ::Entity* entity)
        {
            if (!entity)
            {
                return nullptr;
            }

            auto* transformInterface = GetEntityTransformInterface(entity);
            if (!transformInterface)
            {
                AZ_Error("ROS2FrameComponent", false, "Entity has no transform interface");
                return nullptr;
            }

            const AZ::EntityId parentEntityId = transformInterface->GetParentId();
            if (!parentEntityId.IsValid())
            {
                // Reached top level - no parent entity
                return nullptr;
            }

            AZ::Entity* parentEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, parentEntityId);

            if (!parentEntity)
            {
                AZ_Error("ROS2FrameComponent", false, "Failed to find parent entity with ID: %s", parentEntityId.ToString().c_str());
                return nullptr;
            }

            auto* frameComponent = parentEntity->FindComponent<ROS2FrameComponent>();
            if (!frameComponent)
            {
                // Parent entity has no ROS2Frame, search in ancestors
                return GetFirstROS2FrameAncestor(parentEntity);
            }

            return frameComponent;
        }

        bool ShouldFrameBeDynamic(const AZ::Entity* entity, bool forceDynamic)
        {
            if (forceDynamic)
            {
                return true;
            }

            const bool hasJoints = HasComponentOfType(entity, PhysXJointComponentUuid);
            const bool hasFixedJoints = HasComponentOfType(entity, PhysXFixedJointComponentUuid);
            const bool hasArticulations = HasComponentOfType(entity, PhysXArticulationComponentUuid);

            return (hasJoints && !hasFixedJoints) || hasArticulations;
        }
    } // namespace Internal

    void ROS2FrameComponent::Init()
    {
        m_configuration.m_namespaceConfiguration.Init();
    }

    void ROS2FrameComponent::Activate()
    {
        m_configuration.m_namespaceConfiguration.PopulateNamespace(IsTopLevel(), GetEntity()->GetName());

        if (!m_configuration.m_publishTransform)
        {
            return;
        }

        AZ_TracePrintf("ROS2FrameComponent", "Setting up frame: %s", GetNamespacedFrameID().c_str());

        // Determine if this frame should be dynamic
        if (IsTopLevel() || m_configuration.m_forceDynamic)
        {
            m_configuration.m_isDynamic = true;
        }
        else
        {
            m_configuration.m_isDynamic = Internal::ShouldFrameBeDynamic(GetEntity(), m_configuration.m_forceDynamic);
        }

        const char* transformType = IsDynamic() ? "dynamic" : "static";
        const char* publishTarget = IsDynamic() ? "/tf" : "/tf_static";

        AZ_TracePrintf(
            "ROS2FrameComponent",
            "Publishing %s transform from parent '%s' to child '%s' on %s",
            transformType,
            GetParentFrameID().c_str(),
            GetNamespacedFrameID().c_str(),
            publishTarget);

        m_ros2Transform = AZStd::make_unique<ROS2Transform>(GetParentFrameID(), GetNamespacedFrameID(), IsDynamic());

        // Only start automatic publishing if enabled
        if (m_shouldStartPublishingOnActivate)
        {
            EnableTransformPublishing();
        }

        ROS2FrameComponentBus::Handler::BusConnect(GetEntityId());

        // Register this frame with the frame system
        if (auto* frameRegistration = ROS2FrameRegistrationInterface::Get())
        {
            frameRegistration->RegisterFrame(GetEntityId());
        }
    }

    void ROS2FrameComponent::Deactivate()
    {
        // Unregister this frame from the frame system
        if (auto* frameRegistration = ROS2FrameRegistrationInterface::Get())
        {
            frameRegistration->UnregisterFrame(GetEntityId());
        }

        ROS2FrameComponentBus::Handler::BusDisconnect();
        if (m_configuration.m_publishTransform && IsDynamic())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
        m_ros2Transform.reset();
    }

    void ROS2FrameComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_ros2Transform->Publish(GetFrameTransform());
    }

    AZStd::string ROS2FrameComponent::GetGlobalFrameName() const
    {
        AZStd::string namespacedFrameName;
        ROS2NamesRequestBus::BroadcastResult(
            namespacedFrameName, &ROS2NamesRequests::GetNamespacedName, GetNamespace(), AZStd::string("odom"));

        return namespacedFrameName;
    }

    void ROS2FrameComponent::UpdateNamespaceConfiguration(
        const AZStd::string& ros2Namespace, NamespaceConfiguration::NamespaceStrategy strategy)
    {
        m_configuration.m_namespaceConfiguration.SetNamespace(ros2Namespace, strategy);
    }

    bool ROS2FrameComponent::IsTopLevel() const
    {
        return GetGlobalFrameName() == GetParentFrameID();
    }

    bool ROS2FrameComponent::IsDynamic() const
    {
        return m_configuration.m_isDynamic;
    }

    const ROS2FrameComponent* ROS2FrameComponent::GetParentROS2FrameComponent() const
    {
        return Internal::GetFirstROS2FrameAncestor(GetEntity());
    }

    AZ::Transform ROS2FrameComponent::GetFrameTransform() const
    {
        auto* transformInterface = Internal::GetEntityTransformInterface(GetEntity());
        if (!transformInterface)
        {
            AZ_Error("ROS2FrameComponent", false, "Entity has no transform interface");
            return AZ::Transform::CreateIdentity();
        }

        const auto* parentFrame = GetParentROS2FrameComponent();
        if (!parentFrame)
        {
            // No parent frame - return world transform
            return transformInterface->GetWorldTM();
        }

        auto* ancestorTransformInterface = Internal::GetEntityTransformInterface(parentFrame->GetEntity());
        if (!ancestorTransformInterface)
        {
            AZ_Error("ROS2FrameComponent", false, "Parent frame entity has no transform interface");
            return transformInterface->GetWorldTM();
        }

        // Calculate relative transform: ancestor_from_this = ancestor_from_world * world_from_this
        const AZ::Transform worldFromAncestor = ancestorTransformInterface->GetWorldTM();
        const AZ::Transform worldFromThis = transformInterface->GetWorldTM();
        const AZ::Transform ancestorFromWorld = worldFromAncestor.GetInverse();

        return ancestorFromWorld * worldFromThis;
    }

    AZStd::string ROS2FrameComponent::GetParentFrameID() const
    {
        const auto* parentFrame = GetParentROS2FrameComponent();
        if (parentFrame)
        {
            return parentFrame->GetNamespacedFrameID();
        }

        // No parent ROS2FrameComponent - return global frame name
        return GetGlobalFrameName();
    }

    AZStd::string ROS2FrameComponent::GetNamespacedFrameID() const
    {
        AZStd::string namespacedFrameID;
        ROS2NamesRequestBus::BroadcastResult(
            namespacedFrameID, &ROS2NamesRequests::GetNamespacedName, GetNamespace(), m_configuration.m_frameName);
        return namespacedFrameID;
    }

    void ROS2FrameComponent::SetFrameID(const AZStd::string& frameId)
    {
        m_configuration.m_frameName = frameId;
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        AZStd::string parentNamespace;

        const auto* parentFrame = GetParentROS2FrameComponent();
        if (parentFrame)
        {
            parentNamespace = parentFrame->GetNamespace();
        }

        return m_configuration.m_namespaceConfiguration.GetNamespace(parentNamespace);
    }

    AZ::Name ROS2FrameComponent::GetNamespacedJointName() const
    {
        AZStd::string namespacedJointName;
        ROS2NamesRequestBus::BroadcastResult(
            namespacedJointName, &ROS2NamesRequests::GetNamespacedName, GetNamespace(), m_configuration.m_jointName);
        return AZ::Name(namespacedJointName.c_str());
    }

    void ROS2FrameComponent::SetJointName(const AZStd::string& jointName)
    {
        m_configuration.m_jointName = jointName;
    }

    void ROS2FrameComponent::DisablePublishingOnActivate()
    {
        m_shouldStartPublishingOnActivate = false;
    }

    void ROS2FrameComponent::EnableTransformPublishing()
    {
        if (!m_publishingInitialized && m_configuration.m_publishTransform)
        {
            m_publishingInitialized = true;
            if (IsDynamic())
            {
                AZ::TickBus::Handler::BusConnect();
            }
            else
            {
                // For static transforms, publish once immediately
                m_ros2Transform->Publish(GetFrameTransform());
            }
        }
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2FrameConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, AZ::Component>()->Version(1)->Field("Configuration", &ROS2FrameComponent::m_configuration);
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

    ROS2FrameComponent::ROS2FrameComponent() = default;

    ROS2FrameComponent::ROS2FrameComponent(const ROS2FrameConfiguration& configuration)
        : m_configuration(configuration)
    {
    }

    ROS2FrameConfiguration ROS2FrameComponent::GetConfiguration() const
    {
        return m_configuration;
    }

    void ROS2FrameComponent::SetConfiguration(const ROS2FrameConfiguration& configuration)
    {
        m_configuration = configuration;
    }
} // namespace ROS2
