/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationComponent.h"
#include "Controllers/JointsArticulationControllerComponent.h"
#include "Controllers/JointsPIDControllerComponent.h"
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
    JointsManipulationComponent::JointsManipulationComponent()
    {
    }

    JointsManipulationComponent::JointsManipulationComponent(
        const PublisherConfiguration& configuration, const ManipulationJoints& manipulationJoints)
        : m_jointStatePublisherConfiguration(configuration)
        , m_manipulationJoints(manipulationJoints)
    {
    }

    void JointsManipulationComponent::Activate()
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        JointStatePublisherContext publisherContext;
        publisherContext.m_publisherNamespace = ros2Frame->GetNamespace();
        publisherContext.m_frameId = ros2Frame->GetFrameID();
        publisherContext.m_entityId = GetEntityId();

        m_jointStatePublisher = AZStd::make_unique<JointStatePublisher>(m_jointStatePublisherConfiguration, publisherContext);

        AZ::TickBus::Handler::BusConnect();
        JointsManipulationRequestBus::Handler::BusConnect(GetEntityId());
    }

    void JointsManipulationComponent::Deactivate()
    {
        JointsManipulationRequestBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    ManipulationJoints JointsManipulationComponent::GetJoints()
    {
        return m_manipulationJoints;
    }

    AZ::Outcome<JointPosition, AZStd::string> JointsManipulationComponent::GetJointPosition(const AZStd::string& jointName)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.c_str()));
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

    AZStd::vector<JointPosition> JointsManipulationComponent::GetAllJointsPositions()
    {
        AZStd::vector<JointPosition> positions;
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            positions.push_back(GetJointPosition(jointName).GetValue());
        }
        return positions;
    }

    AZ::Outcome<void, AZStd::string> JointsManipulationComponent::MoveJointToPosition(
        const AZStd::string& jointName, JointPosition position)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.c_str()));
        }
        m_manipulationJoints[jointName].m_restPosition = position;
        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> JointsManipulationComponent::MoveJointsToPositions(
        const AZStd::unordered_map<AZStd::string, JointPosition> positions)
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

    void JointsManipulationComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationComponent, AZ::Component>()
                ->Version(1)
                ->Field("JointStatesPublisherConfiguration", &JointsManipulationComponent::m_jointStatePublisherConfiguration)
                ->Field("ManipulationJoints", &JointsManipulationComponent::m_manipulationJoints);
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
                GetEntityId(),
                &JointsPositionControllerRequests::PositionControl,
                jointName,
                jointInfo,
                currentPosition,
                desiredPosition,
                deltaTime);

            AZ_Warning(
                "JointsManipulationComponent",
                positionControlOutcome,
                "Position control failed for joint %s (%s): %s",
                jointName.c_str(),
                jointInfo.m_entityComponentIdPair.GetEntityId().ToString().c_str(),
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
            AZ_Warning("JointsManipulationComponent", false, "No manipulation joints to handle!");
            AZ::TickBus::Handler::BusDisconnect();
            return;
        }
        m_jointStatePublisher->OnTick(deltaTime);
        MoveToSetPositions(deltaTime);
    }
} // namespace ROS2
