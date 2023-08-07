/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "PhysicsCallbackHandler.h"
#include "AzFramework/Physics/RigidBodyBus.h"
#include "AzFramework/Physics/SimulatedBodies/RigidBody.h"

namespace ROS2::Utils
{
    void PhysicsCallbackHandler::InstallPhysicalCallback()
    {
        m_onSceneSimulationStart = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                OnPhysicsInitialization(sceneHandle);
                m_onSceneSimulationStart.Disconnect();
            });

        m_onSceneSimulationEvent = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                OnPhysicsSimulationFinished(sceneHandle, deltaTime);
            });

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        sceneInterface->RegisterSceneSimulationFinishHandler(sceneHandle, m_onSceneSimulationEvent);
        sceneInterface->RegisterSceneSimulationStartHandler(sceneHandle, m_onSceneSimulationStart);
    }

    void PhysicsCallbackHandler::InstallPhysicalCallback(AZ::EntityId entityId)
    {
        m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        m_onSceneSimulationStart = AzPhysics::SceneEvents::OnSceneSimulationStartHandler(
            [this, entityId](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                AzPhysics::RigidBody* rigidBody = nullptr;
                Physics::RigidBodyRequestBus::EventResult(rigidBody, entityId, &Physics::RigidBodyRequests::GetRigidBody);
                AZ_Assert(rigidBody, "Entity %s does not have rigid body.", entityId.ToString().c_str());
                if (rigidBody)
                {
                    m_bodyHandle = rigidBody->m_bodyHandle;
                    OnPhysicsInitialization(sceneHandle);
                    m_onSceneSimulationStart.Disconnect();
                }
            });

        m_onSceneSimulationEvent = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this](AzPhysics::SceneHandle sceneHandle, float deltaTime)
            {
                if (m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
                {
                    return;
                }
                OnPhysicsSimulationFinished(sceneHandle, deltaTime);
            });

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        sceneInterface->RegisterSceneSimulationFinishHandler(sceneHandle, m_onSceneSimulationEvent);
        sceneInterface->RegisterSceneSimulationStartHandler(sceneHandle, m_onSceneSimulationStart);
    }

    void PhysicsCallbackHandler::RemovePhysicalCallback()
    {
        if (m_onSceneSimulationStart.IsConnected())
        {
            m_onSceneSimulationStart.Disconnect();
        }
        if (m_onSceneSimulationEvent.IsConnected())
        {
            m_onSceneSimulationEvent.Disconnect();
        }
    }
} // namespace ROS2::Utils
