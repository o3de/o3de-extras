/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    void PhysicsBasedSource::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PhysicsBasedSource>()->Version(1)->Field("Source enabled", &PhysicsBasedSource::m_sourceEnabled);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PhysicsBasedSource>("Physics Based Source", "Sensor event source based on physics callback")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PhysicsBasedSource::m_sourceEnabled,
                        "Source enabled",
                        "Enable/disable event source");
            }
        }
    }

    void PhysicsBasedSource::Activate()
    {
        m_onSceneSimulationEvent = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                m_sourceEvent.Signal(sceneHandle, deltaTime);
            });

        auto sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        sceneInterface->RegisterSceneSimulationFinishHandler(sceneHandle, m_onSceneSimulationEvent);
    }

    void PhysicsBasedSource::Deactivate()
    {
        m_onSceneSimulationEvent.Disconnect();
    }

    void PhysicsBasedSource::Configure(const SensorConfiguration& sensorConfiguration)
    {
        m_sourceEnabled = sensorConfiguration.m_publishingEnabled;
    }
} // namespace ROS2