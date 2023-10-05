/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Time/ITime.h>
#include <ROS2/Clock/PhysicallyStableClock.h>

#include <ROS2/ROS2Bus.h>
#include <rclcpp/qos.hpp>

namespace ROS2
{

    void PhysicallyStableClock::Activate()
    {
        auto* systemInterface = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (!systemInterface)
        {
            AZ_Warning("SimulationPhysicalClock", false, "Failed to get AzPhysics::SystemInterface");
            return;
        }
        m_onSceneSimulationEvent = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                m_elapsed += static_cast<double>(deltaTime);
            });

        m_onSceneAdded = AzPhysics::SystemEvents::OnSceneAddedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
                AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
                if (sceneHandle == defaultSceneHandle)
                {
                    AZ_Printf("SimulationPhysicalClock", "Registering clock to default scene");
                    m_elapsed = 0.0;
                    sceneInterface->RegisterSceneSimulationFinishHandler(sceneHandle, m_onSceneSimulationEvent);
                }
            });
        systemInterface->RegisterSceneAddedEvent(m_onSceneAdded);

        m_onSceneRemoved = AzPhysics::SystemEvents::OnSceneRemovedEvent::Handler(
            [this](AzPhysics::SceneHandle sceneHandle)
            {
                auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
                AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
                if (sceneHandle == defaultSceneHandle)
                {
                    AZ_Printf("SimulationPhysicalClock", "Removing clock to default scene");
                    m_onSceneSimulationEvent.Disconnect();
                }
            });
        systemInterface->RegisterSceneRemovedEvent(m_onSceneRemoved);
    }

    void PhysicallyStableClock::Deactivate()
    {
        m_onSceneSimulationEvent.Disconnect();
    };

    builtin_interfaces::msg::Time PhysicallyStableClock::GetROSTimestamp() const
    {
        builtin_interfaces::msg::Time timeStamp;
        timeStamp.sec = static_cast<int32_t>(AZStd::floor(m_elapsed));
        timeStamp.nanosec = static_cast<uint32_t>((m_elapsed - timeStamp.sec) * 1e9);
        return timeStamp;
    }
} // namespace ROS2