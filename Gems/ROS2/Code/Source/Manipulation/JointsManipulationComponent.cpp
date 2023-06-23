/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationComponent.h"
#include "JointStatePublisher.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>

namespace ROS2
{
    namespace Internal
    {
        bool AddHingeJointInfo(
            const PhysX::HingeJointComponent* hingeJointComponent,
            const AZ::Name& jointName,
            JointsManipulationComponent::ManipulationJoints& joints)
        {
            if (joints.find(jointName) != joints.end())
            {
                AZ_Assert(false, "Joint names in hierarchy need to be unique (%s is not)!", jointName.GetCStr());
                return false;
            }

            AZ_Printf("JointsManipulationComponent", "Adding joint info for hinge joint %s\n", jointName.GetCStr());
            JointsManipulationComponent::JointInfo jointInfo;
            jointInfo.m_isArticulation = false;
            jointInfo.m_axis = static_cast<PhysX::ArticulationJointAxis>(0);
            jointInfo.m_entityComponentIdPair = AZ::EntityComponentIdPair(hingeJointComponent->GetEntityId(), hingeJointComponent->GetId());
            joints[jointName] = jointInfo;
            return true;
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

        bool AddArticulationJointInfo(
            const PhysX::ArticulationLinkComponent* articulation,
            const AZ::Name& jointName,
            JointsManipulationComponent::ManipulationJoints& joints)
        {
            auto entityId = articulation->GetEntityId();
            PhysX::ArticulationJointAxis freeAxis;
            bool hasFreeAxis = TryGetFreeArticulationAxis(entityId, freeAxis);
            if (!hasFreeAxis)
            { // Do not add a joint since it is a fixed one
                AZ_Printf("JointsManipulationComponent", "Articulation joint %s is fixed, skipping\n", jointName.GetCStr());
                return false;
            }

            if (joints.find(jointName) != joints.end())
            {
                AZ_Assert(false, "Joint names in hierarchy need to be unique (%s is not)!", jointName.GetCStr());
                return false;
            }

            AZ_Printf("JointsManipulationComponent", "Adding joint info for articulation link %s\n", jointName.GetCStr());
            JointsManipulationComponent::JointInfo jointInfo;
            jointInfo.m_isArticulation = true;
            jointInfo.m_axis = freeAxis;
            jointInfo.m_entityComponentIdPair = AZ::EntityComponentIdPair(entityId, articulation->GetId());
            joints[jointName] = jointInfo;
            return true;
        }
    } // namespace Internal

    void JointsManipulationComponent::Activate()
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());

        JointStatePublisherConfiguration config;
        config.m_publisherNamespace = ros2Frame->GetNamespace();
        config.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        AZStd::string m_topic = "joint_states";
        config.m_frameId = ros2Frame->GetFrameID();
        config.m_frequency = m_frequency;

        m_jointStatePublisher = AZStd::make_unique<JointStatePublisher>(config, GetEntityId());

        AZ::EntityBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        JointsManipulationRequestBus::Handler::BusConnect(GetEntityId());
    }

    void JointsManipulationComponent::Deactivate()
    {
        JointsManipulationRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void JointsManipulationComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect(); // This event is the only one needed.
        InitializeJoints();
    }

    void JointsManipulationComponent::InitializeJoints()
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
                jointAdded = Internal::AddArticulationJointInfo(articulationComponent, jointName, m_manipulationJoints);
            }

            if (jointAdded)
            { // Set the initial / resting position to move to and keep.
                AZ_Warning(
                    "JointsManipulationComponent",
                    m_initialPositions.find(jointName) != m_initialPositions.end(),
                    "No set initial position for joint %s",
                    jointName.GetCStr());
                m_manipulationJoints[jointName].m_restPosition = m_initialPositions[jointName];
            }

            bool foundAtMostOneKind = !(foundArticulation && foundClassicJoint);
            AZ_Assert(foundAtMostOneKind, "Cannot have both classic joints and articulations in same tree");
        }
    }

    JointsManipulationRequests::ManipulationJoints JointsManipulationComponent::GetJoints()
    {
        return m_manipulationJoints;
    }

    AZ::Outcome<JointsManipulationRequests::JointPosition, AZStd::string> JointsManipulationComponent::GetJointPosition(
        const AZ::Name& jointName)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.GetCStr()));
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
            PhysX::JointRequestBus::EventResult(position, jointInfo.m_entityComponentIdPair, &PhysX::JointRequests::GetPosition);
        }
        return AZ::Success(position);
    }

    AZStd::vector<JointsManipulationRequests::JointPosition> JointsManipulationComponent::GetAllJointsPositions()
    {
        AZStd::vector<JointsManipulationRequests::JointPosition> positions;
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            positions.push_back(GetJointPosition(jointName).GetValue());
        }
        return positions;
    }

    AZ::Outcome<void, AZStd::string> JointsManipulationComponent::MoveJointToPosition(
        const AZ::Name& jointName, JointsManipulationComponent::JointPosition position)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.GetCStr()));
        }
        m_manipulationJoints[jointName].m_restPosition = position;
        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> JointsManipulationComponent::MoveJointsToPositions(
        const AZStd::unordered_map<AZ::Name, JointsManipulationRequests::JointPosition> positions)
    {
        for (const auto& [jointName, position] : positions)
        {
            auto outcome = MoveJointToPosition(jointName, position);
            if (!outcome)
            {
                return outcome;
            }
        }
        return AZ::Success();
    }

    void JointsManipulationComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsManipulationComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationComponent, AZ::Component>()
                ->Version(0)
                ->Field("PublishJointStates", &JointsManipulationComponent::m_publishJointState)
                ->Field("JointStatesTopicConfiguration", &JointsManipulationComponent::m_jointStateTopic)
                ->Field("PublishFrequency", &JointsManipulationComponent::m_frequency)
                ->Field("Initial positions", &JointsManipulationComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsManipulationComponent>("JointsManipulationComponent", "Component for a robotic arm (manipulator)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationComponent::m_publishJointState,
                        "Publish Joint States",
                        "Publish JointState messages")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationComponent::m_frequency,
                        "Frequency",
                        "Frequency for JointState messages")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &JointsManipulationComponent::m_jointStateTopic, "Topic", "Topic configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of the manipulator (for all the joints)");
            }
        }
    }

    void JointsManipulationComponent::MoveToSetPositions(float deltaTime)
    {
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            float currentPosition = GetJointPosition(jointName).GetValue();
            float desiredPosition = jointInfo.m_restPosition;

            AZ::Outcome<void, AZStd::string> positionControlOutcome;
            JointsPositionControllerRequestBus::EventResult(
                positionControlOutcome,
                jointInfo.m_entityComponentIdPair.GetEntityId(),
                &JointsPositionControllerRequests::PositionControl,
                jointName,
                jointInfo,
                currentPosition,
                desiredPosition,
                deltaTime);

            AZ_Warning(
                "JointsManipulationComponent",
                positionControlOutcome,
                "Position control failed for joint %s: %s",
                jointName.GetCStr(),
                positionControlOutcome.GetError().c_str());
        }
    }

    void JointsManipulationComponent::Stop()
    {
        for (auto& [jointName, jointInfo] : m_manipulationJoints)
        { // Set all target joint positions to their current positions.
            jointInfo.m_restPosition = GetJointPosition(jointName).GetValue();
        }
    }

    void JointsManipulationComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_manipulationJoints.empty())
        {
            return;
        }
        m_jointStatePublisher->OnTick(deltaTime);
        MoveToSetPositions(deltaTime);
    }
} // namespace ROS2
