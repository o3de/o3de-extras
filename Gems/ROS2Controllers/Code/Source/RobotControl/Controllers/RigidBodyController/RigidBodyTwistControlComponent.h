/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "RigidBodyTwistControlComponentConfig.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2Controllers/RobotControl/Twist/TwistBus.h>

namespace ROS2Controllers
{
    //! A component with a simple handler for Twist type of control (linear and angular velocities).
    //! Velocities are directly applied to a selected body.
    class RigidBodyTwistControlComponent
        : public AZ::Component
        , private TwistNotificationBus::Handler
        , private AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(RigidBodyTwistControlComponent, "{D994FE1A-AA6A-42B9-8B8E-B3B375891F5B}", AZ::Component);
        RigidBodyTwistControlComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void Reflect(AZ::ReflectContext* context);

    private:
        //////////////////////////////////////////////////////////////////////////
        // TwistNotificationBus::Handler overrides
        void TwistReceived(const AZ::Vector3& linear, const AZ::Vector3& angular) override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        void OnSceneSimulationFinish(
            AzPhysics::SceneHandle sceneHandle,
            float fixedDeltaTime); //!< Handler for scene simulation finish event

        AZ::Vector3 m_linearVelocityLocal{ AZ::Vector3::CreateZero() }; //!< Linear velocity in local frame
        AZ::Vector3 m_angularVelocityLocal{ AZ::Vector3::CreateZero() }; //!< Angular velocity in local frame
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneFinishSimHandler; //!< Handler called after every physics sub-step
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle; //!< Handle to the body to apply velocities to
        RigidBodyTwistControlComponentConfig m_config; //!< Configuration for the component
    };
} // namespace ROS2Controllers
