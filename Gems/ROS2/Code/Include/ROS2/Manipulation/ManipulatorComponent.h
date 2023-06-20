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
#include <AzCore/Entity/EntityBus.h>
#include <AzCore/Name/Name.h>

#include "JointStatePublisher.h"
#include <ROS2/Manipulation/ManipulatorRequestBus.h>

namespace ROS2
{
    //! Component responsible for controlling a robotic arm with Articulations or Hinge Joints.
    //! This manipulator component uses simple joint position interface. For trajectory control, see ManipulatorJointTrajectoryComponent.
    class ManipulatorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AZ::EntityBus::Handler
        , public ManipulatorRequestBus::Handler
    {
    public:
        ManipulatorComponent() = default;
        ~ManipulatorComponent() = default;
        AZ_COMPONENT(ManipulatorComponent, "{3da9abfc-0028-4e3e-8d04-4e4440d2e319}", AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void Reflect(AZ::ReflectContext* context);

        // ManipulatorRequestBus::Handler overrides ...
        //! @see ROS2::ManipulatorRequestBus::GetManipulatorJoints
        ManipulatorJoints GetManipulatorJoints() override;
        //! @see ROS2::ManipulatorRequestBus::GetSingleDOFJointPosition
        AZ::Outcome<float, AZStd::string> GetSingleDOFJointPosition(const AZ::Name& jointName) override;
        //! @see ROS2::ManipulatorRequestBus::MoveJointToPosition
        AZ::Outcome<float, AZStd::string> MoveSingleDOFJointToPosition(const AZ::Name& jointName, float position) override;
        //! @see ROS2::ManipulatorRequestBus::Stop
        void Stop() override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        //AZ::EntityBus::Handler overrides
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void InitializeManipulatorJoints();
        void MoveToSetPositions(float deltaTime);

        AZStd::unordered_map<AZ::Name, float> m_initialPositions; // TODO - this should be realized by ManipulatorEditorComponent;
        AZStd::unique_ptr<JointStatePublisher> m_jointStatePublisher;

        bool m_publishJointState { true };
        TopicConfiguration m_jointStateTopic;
        float m_frequency { 25.0f };

        ManipulatorJoints m_manipulatorJoints;
    };
} // namespace ROS2
