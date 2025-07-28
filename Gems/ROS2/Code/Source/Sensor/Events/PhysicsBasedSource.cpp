/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
namespace ROS2
{
    void PhysicsBasedSource::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PhysicsBasedSource>()->Version(1);
        }
    }

    void PhysicsBasedSource::Start()
    {
        m_onSceneSimulationEventHandler.Disconnect();
        m_onSceneSimulationEventHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                builtin_interfaces::msg::Time simulationTime;
                ROS2ClockRequestBus::BroadcastResult(simulationTime, &ROS2ClockRequestBus::Events::GetROSTimestamp);

                const float deltaSimulationTime = ROS2Conversions::GetTimeDifference(m_lastSimulationTime, simulationTime);
                m_sourceEvent.Signal(sceneHandle, deltaSimulationTime);
                m_lastSimulationTime = simulationTime;
            });

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        sceneInterface->RegisterSceneSimulationFinishHandler(sceneHandle, m_onSceneSimulationEventHandler);
    }

    void PhysicsBasedSource::Stop()
    {
        m_onSceneSimulationEventHandler.Disconnect();
    }

    float PhysicsBasedSource::GetDeltaTime([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float deltaTime) const
    {
        return deltaTime;
    }
} // namespace ROS2
