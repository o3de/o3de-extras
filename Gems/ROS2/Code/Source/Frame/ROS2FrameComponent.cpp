/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace Internal
    {
        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity)
        {
            if (!entity)
            {
                AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
                return nullptr;
            }

            auto* interface = Utils::GetGameOrEditorComponent<AzFramework::TransformComponent>(entity);

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

            auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(parentEntity);
            if (component == nullptr)
            { // Parent entity has no ROS2Frame, but there can still be a ROS2Frame in its ancestors
                return GetFirstROS2FrameAncestor(parentEntity);
            }

            // Found the component!
            return component;
        }

        //! Checks whether the entity has a component of the given type
        //! @param entity pointer to entity
        //! @param typeId type of the component
        //! @returns true if entity has component with given type
        static bool CheckIfEntityHasComponentOfType(const AZ::Entity* entity, const AZ::Uuid typeId)
        {
            auto components = AZ::EntityUtils::FindDerivedComponents(entity, typeId);
            return !components.empty();
        }

    } // namespace Internal

    void ROS2FrameComponent::Activate()
    {
        m_configuration.m_namespaceConfiguration.PopulateNamespace(IsTopLevel(), GetEntity()->GetName());

        if (m_configuration.m_publishTransform)
        {
            AZ_TracePrintf("ROS2FrameComponent", "Setting up %s", GetFrameID().data());

            // The frame will always be dynamic if it's a top entity.
            if (IsTopLevel())
            {
                m_configuration.m_isDynamic = true;
            }
            // Otherwise it'll be dynamic when it has joints and it's not a fixed joint.
            else
            {
                const bool hasJoints = Internal::CheckIfEntityHasComponentOfType(
                    m_entity, AZ::Uuid("{B01FD1D2-1D91-438D-874A-BF5EB7E919A8}")); // Physx::JointComponent;
                const bool hasFixedJoints = Internal::CheckIfEntityHasComponentOfType(
                    m_entity, AZ::Uuid("{02E6C633-8F44-4CEE-AE94-DCB06DE36422}")); // Physx::FixedJointComponent
                const bool hasArticulations = Internal::CheckIfEntityHasComponentOfType(
                    m_entity, AZ::Uuid("{48751E98-B35F-4A2F-A908-D9CDD5230264}")); // Physx::ArticulationComponent
                m_configuration.m_isDynamic = (hasJoints && !hasFixedJoints) || hasArticulations;
            }

            AZ_TracePrintf(
                "ROS2FrameComponent",
                "Setting up %s transform between parent %s and child %s to be published %s\n",
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
        if (m_configuration.m_publishTransform)
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

    AZStd::string ROS2FrameComponent::GetGlobalFrameName() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), AZStd::string("odom"));
    }

    void ROS2FrameComponent::UpdateNamespaceConfiguration(
        const AZStd::string& ROS2Namespace, NamespaceConfiguration::NamespaceStrategy strategy)
    {
        m_configuration.m_namespaceConfiguration.SetNamespace(ROS2Namespace, strategy);
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

    AZStd::string ROS2FrameComponent::GetParentFrameID() const
    {
        if (auto parentFrame = GetParentROS2FrameComponent(); parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }
        // If parent entity does not exist or does not have a ROS2FrameComponent, return ROS2 default global frame.
        return GetGlobalFrameName();
    }

    AZStd::string ROS2FrameComponent::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), m_configuration.m_frameName);
    }

    void ROS2FrameComponent::SetFrameID(const AZStd::string& frameId)
    {
        m_configuration.m_frameName = frameId;
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        auto parentFrame = GetParentROS2FrameComponent();
        AZStd::string parentNamespace;
        if (parentFrame != nullptr)
        {
            parentNamespace = parentFrame->GetNamespace();
        }
        return m_configuration.m_namespaceConfiguration.GetNamespace(parentNamespace);
    }

    AZ::Name ROS2FrameComponent::GetJointName() const
    {
        return AZ::Name(ROS2Names::GetNamespacedName(GetNamespace(), m_configuration.m_jointNameString).c_str());
    }

    void ROS2FrameComponent::SetJointName(const AZStd::string& jointNameString)
    {
        m_configuration.m_jointNameString = jointNameString;
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2FrameConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, AZ::Component>()->Version(1)->Field(
                "ROS2FrameConfiguration", &ROS2FrameComponent::m_configuration);
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

    ROS2FrameComponent::ROS2FrameComponent(const ROS2FrameConfiguration& configuration) {
        m_configuration = configuration;
    }
} // namespace ROS2
