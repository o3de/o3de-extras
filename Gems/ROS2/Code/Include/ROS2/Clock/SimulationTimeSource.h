/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ITimeSource.h"
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>

namespace ROS2
{
    //! The SimulationTimeSource provides timestamps calculated using updates from the physics engine.
    //! Simulation paces the clock with stable steps, and as a result, this time source eliminates data jitter.
    //! The simulated time can be faster or slower than the real-time, according to the configuration of the physics engine.
    //! SimulationTimeSource is used as the default time source for the projects.
    //! The simulated clock is incremented with delta times that were simulated by physics.
    //! The time source registers and observes AZ::PhysicsScene and when the
    //! simulation starts, it attaches and starts counting the updates.
    //! It is recommended to use it with high-frequency sensors such as odometry and IMUs.
    class SimulationTimeSource : public ITimeSource
    {
    public:
        virtual ~SimulationTimeSource() = default;

        // ITimeSource overrides ...
        virtual void Activate() override;
        virtual void Deactivate() override;

        //! Get ROS 2 time as ROS2 message.
        //! @see ROS2Requests::GetROSTimestamp() for more details.
        virtual builtin_interfaces::msg::Time GetROSTimestamp() const override;

    private:
        double m_elapsed = 0;
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_onSceneSimulationEvent;
        AzPhysics::SystemEvents::OnSceneAddedEvent::Handler m_onSceneAdded;
        AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler m_onSceneRemoved;
    };
} // namespace ROS2
