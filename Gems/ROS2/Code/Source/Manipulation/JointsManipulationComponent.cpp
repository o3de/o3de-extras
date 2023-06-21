/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointManipulationComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/JointStatePublisher.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>

namespace ROS2
{
    namespace Internal
    {
        void AddHingeJointInfo(const PhysX::HingeJointComponent* hingeJointComponent, const AZ::Name& name, ManipulatorJoint& joints)
        {
            AZ_Assert(joints.find(name) == joints.end(), "Joint names in hierarchy need to be unique (%s is not)!", jointName.GetCStr());
            AZ_Printf("JointManipulationComponent", "Adding joint info for hinge joint %s\n", jointName.GetCStr());
            JointInfo jointInfo;
            jointInfo.m_isArticulation = false;
            jointInfo.m_axis = 0;
            jointInfo.m_entityComponentIdPair = AZ::EntityComponentIdPair(hingeJointComponent->GetEntityId(), hingeJointComponent->GetId());
            joints[name] = jointInfo;
        }

        bool TryGetFreeArticulationAxis(const AZ::EntityId& entityId, PhysX::ArticulationJointAxis& axis)
        {
            for (AZ::u8 i = 0; i <= static_cast<AZ::u8>(PhysX::ArticulationJointAxis::Z); i++)
            {
                PhysX::ArticulationJointMotionType type = PhysX::ArticulationJointMotionType::Locked;
                axis = static_cast<PhysX::ArticulationJointAxis>(i);
                // Use bus to prevent compilation error without PhysX Articulation support.
                PhysX::ArticulationJointRequestBus::EventResult(type, entityId, &PhysX::ArticulationJointRequests::GetMotion, axis);
                if (type != PhysX::ArticulationJointMotionType::Locked)
                {
                    return true;
                }
            }
            return false;
        }

        void AddArticulationJointInfo(
            const PhysX::ArticulationLinkComponent* articulation, const AZ::Name& name, ManipulationJoints& joints)
        {
            auto entityId = articulationComponent->GetEntityId();
            hysX::ArticulationJointAxis freeAxis;
            bool hasFreeAxis = TryGetFreeArticulationAxis(entityId, freeAxis);
            if (!hasFreeAxis)
            { // Do not add a joint since it is a fixed one
                AZ_Printf("JointManipulationComponent", "Articulation joint %s is fixed, skipping\n", jointName.GetCStr());
                return;
            }

            AZ_Assert(joints.find(name) == joints.end(), "Joint names in hierarchy need to be unique (%s is not)!", jointName.GetCStr());
            AZ_Printf("JointManipulationComponent", "Adding joint info for articulation link %s\n", jointName.GetCStr());
            JointInfo jointInfo;
            jointInfo.m_isArticulation = true;
            jointInfo.m_axis = freeAxis;
            jointInfo.m_entityComponentIdPair = AZ::EntityComponentIdPair(entityId, articulationLinkComponent->GetId());
            joints[name] = jointInfo;
        }
    } // namespace Internal

    void JointManipulationComponent::Activate()
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());

        JointStatePublisherConfiguration config;
        config.m_publisherNamespace = ros2Frame->GetNamespace();
        config.m_topicConfiguration = m_topicConfiguration;
        config.m_frameId = ros2Frame->GetFrameId();
        config.m_frequency = m_frequency;

        m_jointStatePublisher = AZStd::make_unique<JointStatePublisher>(config, GetEntityId());

        AZ::EntityBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        JointsManipulationRequestBus::Handler::BusConnect(GetEntityId());
    }

    void JointManipulationComponent::Deactivate()
    {
        JointsManipulationRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
        m_followTrajectoryServer->reset();
    }

    void JointManipulationComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect(); // This event is the only one needed.
        InitializeJoints();
    }

    void JointManipulationComponent::InitializeJoints()
    { // Look for either Articulation Links or Hinge joints in entity hierarchy and collect them into a map.
        bool foundArticulation = false;
        bool foundClassicJoint = false;
        AZStd::vector<AZ::EntityId> descendants;
        AZ::TransformBus::EventResult(descendants, GetEntityId(), &AZ::TransformInterface::GetAllDescendants);
        for (const AZ::EntityId& descendantID : descendants)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, descendantID);
            AZ_Assert(entity, "Unknown entity %s", descendantID.ToString().c_str());

            // If there is a Frame Component, take joint name stored in it.
            auto* frameComponent = entity->FindComponent<ROS2FrameComponent>();
            if (!frameComponent)
            { // Frame Component is required for joints.
                continue;
            }
            const AZ::Name jointName = frameComponent->GetJointName();

            bool jointAdded = false;

            // See if there is a Hinge Joint in the entity, add it to map.
            auto* hingeComponent = entity->FindComponent<PhysX::HingeJointComponent>();
            if (hingeComponent)
            {
                foundClassicJoint = true;
                jointAdded = Internal::AddHingeJointInfo(hingeComponent, jointName, m_manipulationJoints);
            }

            // See if there is an Articulation Link in the entity, add it to map.
            auto* articulationComponent = entity->FindComponent<PhysX::ArticulationLinkComponent>();
            if (articulationComponent)
            {
                foundArticulation = true;
                jointAdded = Internal::AddArticulationJointInfo(hingeComponent, jointName, m_manipulationJoints);
            }

            if (jointAdded)
            { // Set the initial / resting position to move to and keep.
                AZ_Warning(
                    "JointManipulationComponent",
                    m_initialPositions.find(jointName) != m_initialPositions.end(),
                    "No set initial position for joint %s",
                    jointName.GetCStr());
                m_manipulationJoints[jointName].m_restPosition = m_initialPositions[jointName];
            }

            bool foundAtMostOneKind = !(foundArticulation && foundClassicJoint);
            AZ_Assert(foundAtMostOneKind, "Cannot have both classic joints and articulations in same tree");
        }
    }

    ManipulationJoints& JointManipulationComponent::GetJoints()
    {
        return m_manipulationJoints;
    }

    float JointManipulationComponent::GetJointPosition(const AZ::Name& jointName)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure("Joint %s does not exist", jointName.GetCStr()));
        }

        auto jointInfo = m_manipulationJoints.at(jointName);
        float position{ 0 };
        if (jointInfo.m_isArticulation)
        {
            PhysX::ArticulationJointRequestBus::EventResult(
                position,
                jointInfo.m_entityComponentIdPair.GetEntityId(),
                &PhysX::ArticulationJointRequests::GetJointPosition,
                jointInfo.m_axis);
        }
        else
        {
            PhysX::JointRequestBus::EventResult(position, idPair, &PhysX::JointRequests::GetPosition);
        }
        return position;
    }

    void JointManipulationComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void JointManipulationComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ManipulatorService"));
    }

    void JointManipulationComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointManipulationComponent, AZ::Component>()
                ->Version(0)
                ->Field("PublishJointStates", &JointManipulationComponent::m_publishJointState)
                ->Field("JointStatesTopicConfiguration", &JointManipulationComponent::m_jointStateTopic)
                ->Field("PublishFrequency", &JointManipulationComponent::m_frequency)
                ->Field("Initial positions", &JointManipulationComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointManipulationComponent>("JointManipulationComponent", "Component for a robotic arm (manipulator)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointManipulationComponent::m_publishJointState,
                        "Publish Joint States",
                        "Publish JointState messages")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointManipulationComponent::m_frequency,
                        "Frequency",
                        "Frequency for JointState messages")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &JointManipulationComponent::m_jointStateTopic, "Topic", "Topic configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointManipulationComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of the manipulator (for all the joints)");
            }
        }
    }

    void JointManipulationComponent::MoveToSetPositions(float deltaTime)
    {
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            float currentPosition = GetJointPosition(jointName).value();
            float desiredPosition = jointInfo.m_restPosition;

            /*
        if (m_controllerType == Controller::FeedForward)
        {
            desiredVelocity = ComputeFFJointVelocity(
                currentPosition,
                desiredPosition,
                rclcpp::Duration::from_nanoseconds(5e8)); // Dummy forward time reference
        }
        else if (m_controllerType == Controller::PID)
        {
            desiredVelocity = ComputePIDJointVelocity(currentPosition, desiredPosition, deltaTimeNs, jointIndex);
        }
        SetJointVelocity(jointPublisherComponent->GetHierarchyMap().at(jointName), desiredVelocity);
             */
        }
    }

    void JointManipulationComponent::Stop()
    {
        for (auto& [jointName, jointInfo] : m_manipulationJoints)
        { // Set all target joint positions to their current positions.
            jointInfo.m_restPosition = GetJointPosition(jointName).value();
        }
    }

    void JointManipulationComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_manipulationJoints.empty())
        {
            return;
        }
        m_jointStatePublisher->OnTick(deltaTime);
        MoveToSetPositions(deltaTimeNs);
    }
} // namespace ROS2
