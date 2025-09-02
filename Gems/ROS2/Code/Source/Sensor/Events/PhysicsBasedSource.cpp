/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzFramework/Physics/PhysicsSystem.h>

#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/SensorConfiguration.h>

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
        const auto* ros2Interface = ROS2Interface::Get();
        AZ_Assert(ros2Interface, "ROS2 interface is not initialized.");

        m_onSceneSimulationEventHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this, ros2Interface](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                const auto simulationTime = ros2Interface->GetROSTimestamp();
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
