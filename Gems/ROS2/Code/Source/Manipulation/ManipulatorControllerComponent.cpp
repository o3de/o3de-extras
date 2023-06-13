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
#include <ROS2/Manipulation/ManipulatorControllerComponent.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>
namespace ROS2
{
    // ManipulatorControllerComponent class
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    ManipulatorControllerComponent::ManipulatorControllerComponent() = default;
    ManipulatorControllerComponent::~ManipulatorControllerComponent() = default;

    void ManipulatorControllerComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        m_actionServerClass = AZStd::make_unique<FollowJointTrajectoryActionServer>();
        m_actionServerClass->CreateServer(m_ROS2ControllerName);
        InitializePid();
        m_setInitalPose = false;
    }

    void ManipulatorControllerComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_actionServerClass->m_actionServer.reset();
    }

    void ManipulatorControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("JointPublisherService"));
    }

    void ManipulatorControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ManipulatorControllerComponent, AZ::Component>()
                ->Version(0)
                ->Field("ROS2 Controller name", &ManipulatorControllerComponent::m_ROS2ControllerName)
                ->Field("Controller type", &ManipulatorControllerComponent::m_controllerType)
                ->Field("PID Configuration Vector", &ManipulatorControllerComponent::m_pidConfigurationVector)
                ->Field("Initial positions", &ManipulatorControllerComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ManipulatorControllerComponent>(
                      "ManipulatorControllerComponent", "[Controller for a robotic arm (only hinge joints)]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ManipulatorControllerComponent::m_ROS2ControllerName,
                        "ROS2 Controller Name",
                        "It should mirror the name of the ROS2 Controller used as prefix of the ROS2 FollowJointTrajectory Server")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ManipulatorControllerComponent::m_controllerType,
                        "Controller type",
                        "Different controller types to command the joints of the manipulator")
                    ->EnumAttribute(ManipulatorControllerComponent::Controller::FeedForward, "FeedForward")
                    ->EnumAttribute(ManipulatorControllerComponent::Controller::PID, "PID")
                    ->EnumAttribute(ManipulatorControllerComponent::Controller::PhysXArticulation, "PhysX reduced coordinate articulation")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ManipulatorControllerComponent::m_pidConfigurationVector,
                        "PIDs Configuration",
                        "PID controllers configuration (for all the joints)")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ManipulatorControllerComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of the manipulator (for all the joints)");
            }
        }
    }

    void ManipulatorControllerComponent::InitializePid()
    {
        for (auto& pid : m_pidConfigurationVector)
        {
            pid.InitializePid();
        }
    }

    void ManipulatorControllerComponent::InitializeCurrentPosition()
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

    void ManipulatorControllerComponent::InitializePosition()
    {
        auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();
        if (!jointPublisherComponent)
        {
            return;
        }

        for (auto& [jointName, hingeComponent] : jointPublisherComponent->GetHierarchyMap())
        {
            float initialJointPosition = jointPublisherComponent->GetJointPosition(jointName);
            const AZStd::string_view jointNameStr(jointName.GetCStr());
            AZ_Printf("ManipulatorControllerComponent", "Joint name: %s\n", jointNameStr.data());
            if (m_initialPositions.contains(jointNameStr))
            {
                initialJointPosition = m_initialPositions[jointNameStr];

                AZ_Printf(
                    "ManipulatorControllerComponent",
                    "Joint name: %s initialJointPosition : %f \n",
                    jointNameStr.data(),
                    initialJointPosition);
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

    float ManipulatorControllerComponent::ComputeFFJointVelocity(
        const float currentPosition, const float desiredPosition, const rclcpp::Duration& duration) const
    {
        // FeedForward (dummy) method
        float desiredVelocity = (desiredPosition - currentPosition) / duration.seconds();
        return desiredVelocity;
    }

    float ManipulatorControllerComponent::ComputePIDJointVelocity(
        const float currentPosition, const float desiredPosition, const uint64_t& deltaTimeNs, int& jointIndex)
    {
        // PID method
        float error = desiredPosition - currentPosition;
        AZ_Warning("ManipulatorControllerComponent", jointIndex < m_pidConfigurationVector.size(), "Joint index out of range");
        if (jointIndex < m_pidConfigurationVector.size())
        {
            float command = m_pidConfigurationVector.at(jointIndex).ComputeCommand(error, deltaTimeNs);
            return command;
        }
        return 0;
    }

    void ManipulatorControllerComponent::SetJointVelocity(const AZ::EntityComponentIdPair idPair, const float desiredVelocity)
    {
        PhysX::JointRequestBus::Event(idPair, &PhysX::JointRequests::SetVelocity, desiredVelocity);
    }

    void ManipulatorControllerComponent::KeepStillPosition([[maybe_unused]] const uint64_t deltaTimeNs)
    {
        if (!m_keepStillPositionInitialize)
        {
            InitializeCurrentPosition();
            m_keepStillPositionInitialize = true;
        }

        int jointIndex = 0;
        for (auto& [jointName, desiredPosition] : m_jointKeepStillPosition)
        {
            if (jointIndex >= m_pidConfigurationVector.size())
            {
                break;
            }
            auto* jointPublisherComponent = GetEntity()->FindComponent<JointPublisherComponent>();
            if (jointPublisherComponent)
            {
                AZ_Assert(
                    jointPublisherComponent->GetHierarchyMap().contains(jointName),
                    "Joint name %s not found in the hierarchy map",
                    AZStd::string(jointName.GetStringView()).c_str());
                float currentPosition = jointPublisherComponent->GetJointPosition(jointName);
                float desiredVelocity = 0.0f;
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
                jointIndex++;
            }
        }
    }

    void ManipulatorControllerComponent::ExecuteTrajectory([[maybe_unused]] const uint64_t deltaTimeNs)
    {
        // If the trajectory is thoroughly executed set the status to Concluded
        if (m_trajectory.points.size() == 0)
        {
            m_initializedTrajectory = false;
            m_actionServerClass->m_goalStatus = GoalStatus::Concluded;
            AZ_TracePrintf("ManipulatorControllerComponent", "Goal Concluded: all points reached");
            return;
        }

        auto desiredGoal = m_trajectory.points.front();

        rclcpp::Duration timeFromStart =
            rclcpp::Duration(desiredGoal.time_from_start); // arrival time of the current desired trajectory point
        rclcpp::Duration threshold = rclcpp::Duration::from_nanoseconds(1e7);
        rclcpp::Time timeNow = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp()); // current simulation time

        // Jump to the next point if current simulation time is ahead of timeFromStart
        if (m_timeStartingExecutionTraj + timeFromStart <= timeNow + threshold)
        {
            m_trajectory.points.erase(m_trajectory.points.begin());
            ExecuteTrajectory(deltaTimeNs);
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
                AZ_Warning("ManipulatorControllerComponent", false, "Joint name %s not found in the hierarchy map", jointName.GetCStr());
            }
        }
    }

    void ManipulatorControllerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_setInitalPose)
        {
            InitializePosition();
            m_setInitalPose = true;
        }
        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;

        if (m_actionServerClass->m_goalStatus == GoalStatus::Active)
        {
            if (!m_initializedTrajectory)
            {
                m_trajectory = m_actionServerClass->m_goalHandle->get_goal()->trajectory;
                m_timeStartingExecutionTraj = rclcpp::Time(ROS2::ROS2Interface::Get()->GetROSTimestamp());
                m_initializedTrajectory = true;
            }

            ExecuteTrajectory(deltaTimeNs);

            if (m_actionServerClass->m_goalStatus == GoalStatus::Concluded)
            {
                m_actionServerClass->m_goalStatus = GoalStatus::Pending;
                auto result = std::make_shared<FollowJointTrajectory::Result>();
                m_actionServerClass->m_goalHandle->succeed(result);
                m_keepStillPositionInitialize = false;
            }
        }
        else
        {
            KeepStillPosition(deltaTimeNs);
        }
    }

} // namespace ROS2
