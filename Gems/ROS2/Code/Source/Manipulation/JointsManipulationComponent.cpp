/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationComponent.h"
#include "AzCore/Name/Name.h"
#include "Controllers/JointsArticulationControllerComponent.h"
#include "Controllers/JointsPIDControllerComponent.h"
#include "JointStatePublisher.h"
#include "ManipulationUtils.h"
#include "ROS2/Frame/ROS2FrameBus.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>
#include <Source/PrismaticJointComponent.h>

namespace ROS2
{
    namespace Internal
    {
        void Add1DOFJointInfo(const AZ::EntityComponentIdPair& idPair, const AZStd::string& jointName, ManipulationJoints& joints)
        {
            if (joints.find(jointName) != joints.end())
            {
                AZ_Assert(false, "Joint names in hierarchy need to be unique (%s is not)!", jointName.c_str());
                return;
            }
            AZ_Printf("JointsManipulationComponent", "Adding joint info for hinge joint %s\n", jointName.c_str());
            JointInfo jointInfo;
            jointInfo.m_isArticulation = false;
            jointInfo.m_axis = static_cast<PhysX::ArticulationJointAxis>(0);
            jointInfo.m_entityComponentIdPair = idPair;
            joints[jointName] = jointInfo;
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

        void AddArticulationJointInfo(const AZ::EntityComponentIdPair& idPair, const AZStd::string& jointName, ManipulationJoints& joints)
        {
            PhysX::ArticulationJointAxis freeAxis;
            bool hasFreeAxis = TryGetFreeArticulationAxis(idPair.GetEntityId(), freeAxis);
            if (!hasFreeAxis)
            { // Do not add a joint since it is a fixed one
                AZ_Printf("JointsManipulationComponent", "Articulation joint %s is fixed, skipping\n", jointName.c_str());
                return;
            }

            if (joints.find(jointName) != joints.end())
            {
                AZ_Assert(false, "Joint names in hierarchy need to be unique (%s is not)!", jointName.c_str());
                return;
            }

            AZ_Printf("JointsManipulationComponent", "Adding joint info for articulation link %s\n", jointName.c_str());
            JointInfo jointInfo;
            jointInfo.m_isArticulation = true;
            jointInfo.m_axis = freeAxis;
            jointInfo.m_entityComponentIdPair = idPair;
            joints[jointName] = jointInfo;
        }

        ManipulationJoints GetAllEntityHierarchyJoints(const AZ::EntityId& entityId)
        { // Look for either Articulation Links or Hinge joints in entity hierarchy and collect them into a map.
            // Determine kind of joints through presence of appropriate controller
            bool supportsArticulation = false;
            bool supportsClassicJoints = false;
            JointsPositionControllerRequestBus::EventResult(
                supportsArticulation, entityId, &JointsPositionControllerRequests::SupportsArticulation);
            JointsPositionControllerRequestBus::EventResult(
                supportsClassicJoints, entityId, &JointsPositionControllerRequests::SupportsClassicJoints);
            ManipulationJoints manipulationJoints;
            if (!supportsArticulation && !supportsClassicJoints)
            {
                AZ_Warning("JointsManipulationComponent", false, "No suitable Position Controller Component in entity!");
                return manipulationJoints;
            }
            if (supportsArticulation && supportsClassicJoints)
            {
                AZ_Warning("JointsManipulationComponent", false, "Cannot support both classic joint and articulations in one hierarchy");
                return manipulationJoints;
            }

            // Get all descendants and iterate over joints
            AZStd::vector<AZ::EntityId> descendants;
            AZ::TransformBus::EventResult(descendants, entityId, &AZ::TransformInterface::GetEntityAndAllDescendants);
            AZ_Warning("JointsManipulationComponent", descendants.size() > 0, "Entity %s has no descendants!", entityId.ToString().c_str());
            for (const AZ::EntityId& descendantID : descendants)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, descendantID);
                AZ_Assert(entity, "Unknown entity %s", descendantID.ToString().c_str());

                // If there is a Frame Component, take joint name stored in it.
                bool hasFrameComponent = false;
                ROS2FrameComponentBus::EventResult(hasFrameComponent, entity->GetId(), &ROS2FrameComponentBus::Events::IsFrame);
                if (!hasFrameComponent)
                { // Frame Component is required for joints.
                    continue;
                }
                AZ::Name jointNameResult;
                ROS2FrameComponentBus::EventResult(jointNameResult, entity->GetId(), &ROS2FrameComponentBus::Events::GetJointName);
                const AZStd::string jointName(jointNameResult.GetCStr());

                auto* hingeComponent =
                    azrtti_cast<PhysX::JointComponent*>(Utils::GetGameOrEditorComponent<PhysX::HingeJointComponent>(entity));
                auto* prismaticComponent =
                    azrtti_cast<PhysX::JointComponent*>(Utils::GetGameOrEditorComponent<PhysX::PrismaticJointComponent>(entity));
                auto* articulationComponent = Utils::GetGameOrEditorComponent<PhysX::ArticulationLinkComponent>(entity);
                bool classicJoint = hingeComponent || prismaticComponent;
                AZ_Warning(
                    "JointsManipulationComponent",
                    (classicJoint && supportsClassicJoints) || !classicJoint,
                    "Found classic joints but the controller does not support them!");
                AZ_Warning(
                    "JointsManipulationComponent",
                    (articulationComponent && supportsArticulation) || !articulationComponent,
                    "Found articulations but the controller does not support them!");

                // See if there is a Hinge Joint in the entity, add it to map.
                if (supportsClassicJoints && hingeComponent)
                {
                    auto idPair = AZ::EntityComponentIdPair(hingeComponent->GetEntityId(), hingeComponent->GetId());
                    Internal::Add1DOFJointInfo(idPair, jointName, manipulationJoints);
                }

                // See if there is a Prismatic Joint in the entity, add it to map.
                if (supportsClassicJoints && prismaticComponent)
                {
                    auto idPair = AZ::EntityComponentIdPair(prismaticComponent->GetEntityId(), prismaticComponent->GetId());
                    Internal::Add1DOFJointInfo(idPair, jointName, manipulationJoints);
                }

                // See if there is an Articulation Link in the entity, add it to map.
                if (supportsArticulation && articulationComponent)
                {
                    auto idPair = AZ::EntityComponentIdPair(articulationComponent->GetEntityId(), articulationComponent->GetId());
                    Internal::AddArticulationJointInfo(idPair, jointName, manipulationJoints);
                }
            }
            return manipulationJoints;
        }

        void SetInitialPositions(ManipulationJoints& manipulationJoints, const AZStd::unordered_map<AZStd::string, float>& initialPositions)
        {
            // Set the initial / resting position to move to and keep.
            for (const auto& [jointName, jointInfo] : manipulationJoints)
            {
                if (initialPositions.contains(jointName))
                {
                    manipulationJoints[jointName].m_restPosition = initialPositions.at(jointName);
                }
                else
                {
                    AZ_Warning("JointsManipulationComponent", false, "No set initial position for joint %s", jointName.c_str());
                }
            }
        }
    } // namespace Internal

    JointsManipulationComponent::JointsManipulationComponent()
    {
    }

    JointsManipulationComponent::JointsManipulationComponent(
        const PublisherConfiguration& configuration, const AZStd::unordered_map<AZStd::string, JointPosition>& initialPositions)
        : m_jointStatePublisherConfiguration(configuration)
        , m_initialPositions(initialPositions)
    {
    }

    void JointsManipulationComponent::Activate()
    {
        AZStd::string frameNamespace;
        AZStd::string frameId;
        ROS2FrameComponentBus::EventResult(frameNamespace, GetEntityId(), &ROS2FrameComponentBus::Events::GetNamespace);
        ROS2FrameComponentBus::EventResult(frameId, GetEntityId(), &ROS2FrameComponentBus::Events::GetFrameID);
        JointStatePublisherContext publisherContext;
        publisherContext.m_publisherNamespace = frameNamespace;
        publisherContext.m_frameId = frameId;
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

    AZ::Outcome<JointPosition, AZStd::string> JointsManipulationComponent::GetJointPosition(const JointInfo& jointInfo)
    {
        float position{ 0.f };
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

    AZ::Outcome<JointPosition, AZStd::string> JointsManipulationComponent::GetJointPosition(const AZStd::string& jointName)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.c_str()));
        }

        auto jointInfo = m_manipulationJoints.at(jointName);

        return GetJointPosition(jointInfo);
    }

    AZ::Outcome<JointVelocity, AZStd::string> JointsManipulationComponent::GetJointVelocity(const JointInfo& jointInfo)
    {
        float velocity{ 0.f };
        if (jointInfo.m_isArticulation)
        {
            PhysX::ArticulationJointRequestBus::EventResult(
                velocity,
                jointInfo.m_entityComponentIdPair.GetEntityId(),
                &PhysX::ArticulationJointRequests::GetJointVelocity,
                jointInfo.m_axis);
        }
        else
        {
            PhysX::JointRequestBus::EventResult(velocity, jointInfo.m_entityComponentIdPair, &PhysX::JointRequests::GetVelocity);
        }
        return AZ::Success(velocity);
    }

    AZ::Outcome<JointVelocity, AZStd::string> JointsManipulationComponent::GetJointVelocity(const AZStd::string& jointName)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.c_str()));
        }

        auto jointInfo = m_manipulationJoints.at(jointName);
        return GetJointVelocity(jointInfo);
    }

    JointsManipulationRequests::JointsPositionsMap JointsManipulationComponent::GetAllJointsPositions()
    {
        JointsManipulationRequests::JointsPositionsMap positions;
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            positions[jointName] = GetJointPosition(jointInfo).GetValue();
        }
        return positions;
    }

    JointsManipulationRequests::JointsVelocitiesMap JointsManipulationComponent::GetAllJointsVelocities()
    {
        JointsManipulationRequests::JointsVelocitiesMap velocities;
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            velocities[jointName] = GetJointVelocity(jointInfo).GetValue();
        }
        return velocities;
    }

    AZ::Outcome<JointEffort, AZStd::string> JointsManipulationComponent::GetJointEffort(const JointInfo& jointInfo)
    {
        auto jointStateData = Utils::GetJointState(jointInfo);

        return AZ::Success(jointStateData.effort);
    }

    AZ::Outcome<JointEffort, AZStd::string> JointsManipulationComponent::GetJointEffort(const AZStd::string& jointName)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.c_str()));
        }

        auto jointInfo = m_manipulationJoints.at(jointName);
        return GetJointEffort(jointInfo);
    }

    JointsManipulationRequests::JointsEffortsMap JointsManipulationComponent::GetAllJointsEfforts()
    {
        JointsManipulationRequests::JointsEffortsMap efforts;
        for (const auto& [jointName, jointInfo] : m_manipulationJoints)
        {
            efforts[jointName] = GetJointEffort(jointInfo).GetValue();
        }
        return efforts;
    }

    AZ::Outcome<void, AZStd::string> JointsManipulationComponent::SetMaxJointEffort(const AZStd::string& jointName, JointEffort maxEffort)
    {
        if (!m_manipulationJoints.contains(jointName))
        {
            return AZ::Failure(AZStd::string::format("Joint %s does not exist", jointName.c_str()));
        }

        auto jointInfo = m_manipulationJoints.at(jointName);

        if (jointInfo.m_isArticulation)
        {
            PhysX::ArticulationJointRequestBus::Event(
                jointInfo.m_entityComponentIdPair.GetEntityId(),
                &PhysX::ArticulationJointRequests::SetMaxForce,
                jointInfo.m_axis,
                maxEffort);
        }

        return AZ::Success();
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
        const JointsManipulationRequests::JointsPositionsMap& positions)
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
                ->Field("InitialJointPosition", &JointsManipulationComponent::m_initialPositions);
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
        { // Set all target joint positions to their current positions. There is no need to check if the outcome is successful, because
          // jointName is always valid.
            jointInfo.m_restPosition = GetJointPosition(jointName).GetValue();
        }
    }

    void JointsManipulationComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_manipulationJoints.empty())
        {
            m_manipulationJoints = Internal::GetAllEntityHierarchyJoints(GetEntityId());
            Internal::SetInitialPositions(m_manipulationJoints, m_initialPositions);
            if (m_manipulationJoints.empty())
            {
                AZ_Warning("JointsManipulationComponent", false, "No manipulation joints to handle!");
                AZ::TickBus::Handler::BusDisconnect();
                return;
            }
            m_jointStatePublisher->InitializePublisher(GetEntityId());
        }
        MoveToSetPositions(deltaTime);
    }
} // namespace ROS2
