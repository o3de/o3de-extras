/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once
#include "SimulationClock.h"
#include <AzCore/std/chrono/chrono.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <builtin_interfaces/msg/time.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace ROS2
{
    //! Simulation clock which changes source to physics of timestamps that are provided by \ref GetROSTimestamp.
    //! The simulated clock is incremented with delta times that were simulated by physics.
    //! Clock register and observes AZ::PhysicsScene and when simulation starts, it attaches and starts to count updates.
    //! It is recommended to use with high-frequency sensors such as odometry and IMUs.
    class PhysicallyStableClock : public SimulationClock
    {
    public:
        // SimulationClock overrides ...
        void Activate() override;
        void Deactivate() override;
        builtin_interfaces::msg::Time GetROSTimestamp() const override;

        virtual ~PhysicallyStableClock() = default;

    private:
        double m_elapsed = 0;
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_onSceneSimulationEvent;
        AzPhysics::SystemEvents::OnSceneAddedEvent::Handler m_onSceneAdded;
        AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler m_onSceneRemoved;
    };
} // namespace ROS2