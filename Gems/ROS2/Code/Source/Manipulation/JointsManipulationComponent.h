/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Name/Name.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/utility/pair.h>

#include "JointStatePublisher.h"
#include <ROS2/Manipulation/JointsManipulationRequests.h>

namespace ROS2
{
    //! Component responsible for controlling a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    //! This manipulator component uses simple joint position interface. For trajectory control, see JointsTrajectoryComponent.
    class JointsManipulationComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public JointsManipulationRequestBus::Handler
    {
    public:
        JointsManipulationComponent();
        JointsManipulationComponent(
            const PublisherConfiguration& publisherConfiguration,
            const AZStd::vector<AZStd::pair<AZStd::string, float>>& initialPositions);
        ~JointsManipulationComponent() = default;
        AZ_COMPONENT(JointsManipulationComponent, "{3da9abfc-0028-4e3e-8d04-4e4440d2e319}", AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // JointsManipulationRequestBus::Handler overrides ...
        //! @see ROS2::JointsManipulationRequestBus::GetJoints
        ManipulationJoints GetJoints() override;
        //! @see ROS2::JointsManipulationRequestBus::GetJointPosition
        AZ::Outcome<JointPosition, AZStd::string> GetJointPosition(const AZStd::string& jointName) override;
        //! @see ROS2::JointsManipulationRequestBus::GetJointVelocity
        AZ::Outcome<JointVelocity, AZStd::string> GetJointVelocity(const AZStd::string& jointName) override;
        //! @see ROS2::JointsManipulationRequestBus::GetAllJointsPositions
        JointsPositionsMap GetAllJointsPositions() override;
        //! @see ROS2::JointsManipulationRequestBus::GetAllJointsVelocities
        JointsVelocitiesMap GetAllJointsVelocities() override;
        //! @see ROS2::JointsManipulationRequestBus::GetJointEffort
        AZ::Outcome<JointEffort, AZStd::string> GetJointEffort(const AZStd::string& jointName) override;
        //! @see ROS2::JointsManipulationRequestBus::GetAllJointsEfforts
        JointsEffortsMap GetAllJointsEfforts() override;
        //! @see ROS2::JointsManipulationRequestBus::SetMaxJointEffort
        AZ::Outcome<void, AZStd::string> SetMaxJointEffort(const AZStd::string& jointName, JointEffort maxEffort);
        //! @see ROS2::JointsManipulationRequestBus::MoveJointsToPositions
        AZ::Outcome<void, AZStd::string> MoveJointsToPositions(const JointsPositionsMap& positions) override;
        //! @see ROS2::JointsManipulationRequestBus::MoveJointToPosition
        AZ::Outcome<void, AZStd::string> MoveJointToPosition(const AZStd::string& jointName, JointPosition position) override;
        //! @see ROS2::JointsManipulationRequestBus::Stop
        void Stop() override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void MoveToSetPositions(float deltaTime);

        AZStd::string GetManipulatorNamespace() const;

        AZ::Outcome<JointPosition, AZStd::string> GetJointPosition(const JointInfo& jointInfo);
        AZ::Outcome<JointVelocity, AZStd::string> GetJointVelocity(const JointInfo& jointInfo);
        AZ::Outcome<JointEffort, AZStd::string> GetJointEffort(const JointInfo& jointInfo);

        AZStd::unique_ptr<JointStatePublisher> m_jointStatePublisher;
        PublisherConfiguration m_jointStatePublisherConfiguration;
        ManipulationJoints m_manipulationJoints; //!< Map of JointInfo where the key is a joint name (with namespace included)
        AZStd::vector<AZStd::pair<AZStd::string, float>> 
            m_initialPositions; //!< Initial positions per joint name (without namespace included)
    };
} // namespace ROS2
