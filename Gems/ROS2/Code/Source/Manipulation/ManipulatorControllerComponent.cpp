/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FollowJointTrajectoryActionServer.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/functional.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/JointPublisherComponent.h>
#include <ROS2/Manipulation/ManipulatorComponent.h>
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
            AZ_Printf("ManipulatorComponent", "Adding joint info for hinge joint %s\n", jointName.GetCStr());
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

        void AddArticulationJointInfo(const PhysX::ArticulationLinkComponent* articulation, const AZ::Name& name, ManipulatorJoints& joints)
        {
            auto entityId = articulationComponent->GetEntityId();
            hysX::ArticulationJointAxis freeAxis;
            bool hasFreeAxis = TryGetFreeArticulationAxis(entityId, freeAxis);
            if (!hasFreeAxis)
            { // Do not add a joint since it is a fixed one
                AZ_Printf("ManipulatorComponent", "Articulation joint %s is fixed, skipping\n", jointName.GetCStr());
                return;
            }

            AZ_Assert(joints.find(name) == joints.end(), "Joint names in hierarchy need to be unique (%s is not)!", jointName.GetCStr());
            AZ_Printf("ManipulatorComponent", "Adding joint info for articulation link %s\n", jointName.GetCStr());
            JointInfo jointInfo;
            jointInfo.m_isArticulation = true;
            jointInfo.m_axis = freeAxis;
            jointInfo.m_entityComponentIdPair = AZ::EntityComponentIdPair(entityId, articulationLinkComponent->GetId());
            joints[name] = jointInfo;
        }
    } // namespace Internal

    void ManipulatorComponent::Activate()
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto entityNamespace = ros2Frame->GetNamespace();
        auto baseActionName = m_ROS2ControllerName.append("/follow_joint_trajectory").data();
        auto actionName = ROS2Names::GetNamespacedName(entityNamespace, baseActionName);
        m_followTrajectoryServer = AZStd::make_unique<FollowJointTrajectoryActionServer>(actionName);

        AZ::EntityBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        ManipulatorRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ManipulatorComponent::Deactivate()
    {
        ManipulatorRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
        m_followTrajectoryServer->reset();
    }

    void ManipulatorComponent::OnEntityActivated([[maybe_unused]] const AZ::EntityId& entityId)
    {
        AZ::EntityBus::Handler::BusDisconnect(); // This event is the only one needed.
        InitializeManipulatorJoints();
    }

    void ManipulatorComponent::InitializeManipulatorJoints()
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
                jointAdded = Internal::AddHingeJointInfo(hingeComponent, jointName, m_manipulatorJoints);
            }

            // See if there is an Articulation Link in the entity, add it to map.
            auto* articulationComponent = entity->FindComponent<PhysX::ArticulationLinkComponent>();
            if (articulationComponent)
            {
                foundArticulation = true;
                jointAdded = Internal::AddArticulationJointInfo(hingeComponent, jointName, m_manipulatorJoints);
            }

            if (jointAdded)
            { // Set the initial / resting position to move to and keep.
                AZ_Warning(
                    "ManipulatorComponent",
                    m_initialPositions.find(jointName) != m_initialPositions.end(),
                    "No set initial position for joint %s",
                    jointName.GetCStr());
                m_manipulatorJoints[jointName].m_restPosition = m_initialPositions[jointName];
            }

            bool foundAtMostOneKind = !(foundArticulation && foundClassicJoint);
            AZ_Assert(foundAtMostOneKind, "Cannot have both classic joints and articulations in same tree");
        }
    }

    ManipulatorJoints& ManipulatorComponent::GetManipulatorJoints()
    {
        return m_manipulatorJoints;
    }

    float ManipulatorComponent::GetSingleDOFJointPosition(const AZ::Name& jointName)
    {
        if (!m_manipulatorJoints.contains(jointName))
        {
            return AZ::Failure("Joint %s does not exist", jointName.GetCStr()));
        }

        auto jointInfo = m_manipulatorJoints.at(jointName);
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

    AZ::Outcome<void, AZStd::string> ManipulatorComponent::StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal);
    {
        if (IsBusy())
        {
            return AZ::Failure("Another trajectory goal is executing. Wait for completion or cancel it");
        }
        m_trajectory = trajectoryGoal->trajectory;
        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> ManipulatorComponent::CancelTrajectory(TrajectoryResultPtr result)
    {
        m_trajectory.clear(); // Simply empty the trajectory
        m_followTrajectoryServer->CancelGoal(result);
        return AZ::Success();
    }

    GoalStatus ManipulatorComponent::GetGoalStatus()
    {
        return m_followTrajectoryServer->GetGoalStatus();
    }

    void ManipulatorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointPublisherService"));
    }

    void ManipulatorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorComponent, AZ::Component>()
                ->Version(0)
                ->Field("Action name", &ManipulatorComponent::m_actionName)
                ->Field("Controller type", &ManipulatorComponent::m_controllerType)
                ->Field("PID Configuration Vector", &ManipulatorComponent::m_pidConfigurationVector)
                ->Field("Initial positions", &ManipulatorComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorComponent>("ManipulatorComponent", "Component to control a robotic arm")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ManipulatorComponent::m_actionName,
                        "Action Name",
                        "Name the follow trajectory action server to accept movement commands")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ManipulatorComponent::m_controllerType,
                        "Controller type",
                        "How to realize the commanded movement")
                    ->EnumAttribute(ManipulatorComponent::Controller::PhysXArticulation, "PhysX reduced coordinate articulation")
                    ->EnumAttribute(ManipulatorComponent::Controller::PID, "PID")
                    ->EnumAttribute(ManipulatorComponent::Controller::FeedForward, "FeedForward")
                    // TODO - validate whether Articulation is available - look for Articulation Root in the editor component
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ManipulatorComponent::m_pidConfigurationVector,
                        "PIDs Configuration",
                        "PID controllers configuration (for all the joints)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ManipulatorComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of the manipulator (for all the joints)");
            }
        }
    }

    void ManipulatorComponent::InitializePid()
    {
        for (auto& pid : m_pidConfigurationVector)
        {
            pid.InitializePid();
        }
    }

    void ManipulatorComponent::InitializeCurrentPosition()
    {
        auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();
        if (!jointPublisherComponent)
        {
            for (auto& [jointName, hingeComponent] : jointPublisherComponent->GetHierarchyMap())
            {
                m_jointKeepStillPosition[jointName] = jointPublisherComponent->GetJointPosition(jointName);
            }
        }
    }

    void ManipulatorComponent::MoveToInitialPositions()
    {
        for (const auto& [jointName, jointInfo] : m_manipulatorJoints)
        {
        }

        for (auto& [jointName, hingeComponent] : jointPublisherComponent->GetHierarchyMap())
        {
            float initialJointPosition = jointPublisherComponent->GetJointPosition(jointName);
            const AZStd::string_view jointNameStr(jointName.GetCStr());
            AZ_Printf("ManipulatorComponent", "Joint name: %s\n", jointNameStr.data());
            if (m_initialPositions.contains(jointNameStr))
            {
                initialJointPosition = m_initialPositions[jointNameStr];

                AZ_Printf("ManipulatorComponent", "Joint name: %s initialJointPosition : %f \n", jointNameStr.data(), initialJointPosition);
            }
            if (m_controllerType == Controller::PhysXArticulation)
            {
                const auto articulatedAxis = jointPublisherComponent->GetArticulationFreeAxis(jointName);
                PhysX::ArticulationJointRequestBus::Event(
                    hingeComponent.GetEntityId(), &PhysX::ArticulationJointRequests::SetDriveTarget, articulatedAxis, initialJointPosition);
            }
            else
            {
                m_jointKeepStillPosition[jointName] = initialJointPosition;
            }
        }
    }

    float ManipulatorComponent::ComputeFFJointVelocity(
        const float currentPosition, const float desiredPosition, const rclcpp::Duration& duration) const
    {
        // FeedForward (dummy) method
        float desiredVelocity = (desiredPosition - currentPosition) / duration.seconds();
        return desiredVelocity;
    }

    float ManipulatorComponent::ComputePIDJointVelocity(
        const float currentPosition, const float desiredPosition, const uint64_t& deltaTimeNs, int& jointIndex)
    {
        // PID method
        float error = desiredPosition - currentPosition;
        AZ_Warning("ManipulatorComponent", jointIndex < m_pidConfigurationVector.size(), "Joint index out of range");
        if (jointIndex < m_pidConfigurationVector.size())
        {
            float command = m_pidConfigurationVector.at(jointIndex).ComputeCommand(error, deltaTimeNs);
            return command;
        }
        return 0;
    }

    void ManipulatorComponent::SetJointVelocity(const AZ::EntityComponentIdPair idPair, const float desiredVelocity)
    {
        PhysX::JointRequestBus::Event(idPair, &PhysX::JointRequests::SetVelocity, desiredVelocity);
    }

    void ManipulatorComponent::MoveToKeepStillPositions(const uint64_t deltaTimeNs)
    {
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            float currentPosition = GetSingleDOFJointPosition(jointName).value();
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
}

void ManipulatorComponent::FollowTrajectory([[maybe_unused]] const uint64_t deltaTimeNs)
{
    // If the trajectory is thoroughly executed set the status to Concluded
    if (m_trajectory.points.size() == 0)
    {
        m_initializedTrajectory = false;
        m_followTrajectoryServer->m_goalStatus = GoalStatus::Concluded;
        AZ_TracePrintf("ManipulatorComponent", "Goal Concluded: all points reached");
        return;
    }

    auto desiredGoal = m_trajectory.points.front();

    rclcpp::Duration timeFromStart = rclcpp::Duration(desiredGoal.time_from_start); // arrival time of the current desired trajectory point
    rclcpp::Duration threshold = rclcpp::Duration::from_nanoseconds(1e7);
    rclcpp::Time timeNow = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp()); // current simulation time

    // Jump to the next point if current simulation time is ahead of timeFromStart
    if (m_timeStartingExecutionTraj + timeFromStart <= timeNow + threshold)
    {
        m_trajectory.points.erase(m_trajectory.points.begin());
        FollowTrajectory(deltaTimeNs);
        return;
    }

    for (int jointIndex = 0; jointIndex < m_trajectory.joint_names.size(); jointIndex++)
    {
        const auto& jointNameStr = m_trajectory.joint_names[jointIndex];
        const auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();

        if (!jointPublisherComponent)
        {
            continue;
        }
        const auto& hierarchy = jointPublisherComponent->GetHierarchyMap();
        const auto jointName = AZ::Name(jointNameStr.c_str());
        if (hierarchy.contains(jointName))
        {
            const AZ::EntityComponentIdPair& componentId = hierarchy.at(jointName);
            float currentPosition = jointPublisherComponent->GetJointPosition(jointName);
            float desiredPosition = desiredGoal.positions[jointIndex];
            float desiredVelocity = 0.0f;
            if (m_controllerType == Controller::FeedForward)
            {
                desiredVelocity =
                    ComputeFFJointVelocity(currentPosition, desiredPosition, m_timeStartingExecutionTraj + timeFromStart - timeNow);
                SetJointVelocity(componentId, desiredVelocity);
            }
            else if (m_controllerType == Controller::PID)
            {
                desiredVelocity = ComputePIDJointVelocity(currentPosition, desiredPosition, deltaTimeNs, jointIndex);
                SetJointVelocity(componentId, desiredVelocity);
            }
            else if (m_controllerType == Controller::PhysXArticulation)
            {
                const auto articulatedAxis = jointPublisherComponent->GetArticulationFreeAxis(jointName);
                PhysX::ArticulationJointRequestBus::Event(
                    componentId.GetEntityId(), &PhysX::ArticulationJointRequests::SetDriveTarget, articulatedAxis, desiredPosition);
            }
        }
        else
        {
            AZ_Warning("ManipulatorComponent", false, "Joint name %s not found in the hierarchy map", jointName.GetCStr());
        }
    }
}

void ManipulatorComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
{
    if (m_manipulatorJoints.empty())
    { // Nothing to do
        return;
    }

    const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
    MoveToSetPositions(deltaTimeNs);

    /*
    if (GetGoalStatus() != GoalStatus::Executing)
    {

    if (!m_initializedTrajectory)
    {
        m_trajectory = m_followTrajectoryServer->m_goalHandle->get_goal()->trajectory;
        m_timeStartingExecutionTraj = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp());
        m_initializedTrajectory = true;
    }

    FollowTrajectory(deltaTimeNs);

    if (m_trajectory.points.size() == 0)
    {   // Goal reached
        m_initializedTrajectory = false;
        AZ_TracePrintf("ManipulatorComponent", "Goal Concluded: all points reached");
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        m_followTrajectoryServer->GoalSuccess(result);
        m_keepStillPositionInitialize = false;
    }
    */
}

} // namespace ROS2
