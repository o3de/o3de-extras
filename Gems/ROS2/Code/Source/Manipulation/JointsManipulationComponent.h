/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Name/Name.h>

#include "JointStatePublisher.h"
#include <ROS2/Manipulation/JointsManipulationRequests.h>

namespace ROS2
{
    //! Component responsible for controlling a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    //! This manipulator component uses simple joint position interface. For trajectory control, see JointsTrajectoryComponent.
    class JointsManipulationComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AZ::EntityBus::Handler
        , public JointsManipulationRequestBus::Handler
    {
    public:
        JointsManipulationComponent() = default;
        ~JointsManipulationComponent() = default;
        AZ_COMPONENT(JointsManipulationComponent, "{3da9abfc-0028-4e3e-8d04-4e4440d2e319}", AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void Reflect(AZ::ReflectContext* context);

        // JointsManipulationRequestBus::Handler overrides ...
        //! @see ROS2::JointsManipulationRequestBus::GetJoints
        ManipulationJoints GetJoints() override;
        //! @see ROS2::JointsManipulationRequestBus::GetJointPosition
        AZ::Outcome<JointPosition, AZStd::string> GetJointPosition(const AZ::Name& jointName) override;
        //! @see ROS2::JointsManipulationRequestBus::GetAllJointsPositions
        AZStd::vector<JointPosition> GetAllJointsPositions() override;
        //! @see ROS2::JointsManipulationRequestBus::MoveJointsToPosition
        AZ::Outcome<void, AZStd::string> MoveJointsToPositions(
            const AZStd::unordered_map<AZ::Name, JointPosition> positions) override;
        //! @see ROS2::JointsManipulationRequestBus::MoveJointToPosition
        AZ::Outcome<void, AZStd::string> MoveJointToPosition(const AZ::Name& jointName, JointPosition position) override;
        //! @see ROS2::JointsManipulationRequestBus::Stop
        void Stop() override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::EntityBus::Handler overrides
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void InitializeJoints();
        void MoveToSetPositions(float deltaTime);

        AZStd::unordered_map<AZ::Name, float> m_initialPositions; // TODO - this should be realized by an Editor Component;
        AZStd::unique_ptr<JointStatePublisher> m_jointStatePublisher;

        bool m_publishJointState{ true };
        TopicConfiguration m_jointStateTopic;
        float m_frequency{ 25.0f };

        ManipulationJoints m_manipulationJoints;
    };
} // namespace ROS2
