/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameController.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
namespace ROS2
{
    namespace Internal
    {
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
        ROS2FrameComponentBase::Activate();
        m_controller.PopulateNamespace(IsTopLevel(), GetEntity()->GetName());

        if (m_controller.GetPublishTransform())
        {
            AZ_TracePrintf("ROS2FrameComponent", "Setting up %s", GetFrameID().data());

            // The frame will always be dynamic if it's a top entity.
            if (IsTopLevel())
            {
                m_controller.SetIsDynamic(true);
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
                m_controller.SetIsDynamic((hasJoints && !hasFixedJoints) || hasArticulations);
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
                m_ros2Transform->Publish(m_controller.GetFrameTransform());
            }
        }
    }

    void ROS2FrameComponent::Deactivate()
    {
        ROS2FrameComponentBase::Deactivate();
        if (m_controller.GetPublishTransform())
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
        m_ros2Transform->Publish(m_controller.GetFrameTransform());
    }

    AZStd::string ROS2FrameComponent::GetGlobalFrameName() const
    {
        return m_controller.GetGlobalFrameName();
    }

    void ROS2FrameComponent::UpdateNamespaceConfiguration(const AZStd::string& ns, NamespaceConfiguration::NamespaceStrategy strategy)
    {
        m_namespaceConfiguration.SetNamespace(ns, strategy);
    }

    bool ROS2FrameComponent::IsTopLevel() const
    {
        return m_controller.IsTopLevel();
    }

    bool ROS2FrameComponent::IsDynamic() const
    {
        return m_controller.IsDynamic();
    }

    AZ::Transform ROS2FrameComponent::GetFrameTransform() const
    {
        return m_controller.GetFrameTransform();
    }

    AZStd::string ROS2FrameComponent::GetParentFrameID() const
    {
        return m_controller.GetParentFrameID();
    }

    AZStd::string ROS2FrameComponent::GetFrameID() const
    {
        return m_controller.GetFrameID();
    }

    void ROS2FrameComponent::SetFrameID(const AZStd::string& frameId)
    {
        m_controller.SetFrameID(frameId);
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        return m_controller.GetNamespace();
    }

    AZ::Name ROS2FrameComponent::GetJointName() const
    {
        return m_controller.GetJointName();
    }

    void ROS2FrameComponent::SetJointName(const AZStd::string& jointNameString)
    {
        m_controller.SetJointName(jointNameString);
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2FrameComponentBase::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, ROS2FrameComponentBase>()->Version(1);

            // if (AZ::EditContext* ec = serialize->GetEditContext())
            // {
            //     ec->Class<ROS2FrameComponent>("ROS2 Frame", "[ROS2 Frame component]")
            //         ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
            //         ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
            //         ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
            //         ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            // }
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

    ROS2FrameComponent::ROS2FrameComponent(const AZStd::string& frameId)
    {
        SetFrameID(frameId);
    }

    ROS2FrameComponent::ROS2FrameComponent(const ROS2FrameConfiguration& config)
    {
        SetConfiguration(config);
    }
} // namespace ROS2
