/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/Time/ITime.h>
#include <ROS2/Clock/SimulationTimeSource.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    namespace
    {
        constexpr AZStd::string_view ResetTimestampOnLevelReload = "/O3DE/ROS2/SteadyClock/ResetTimestampOnLevelReload";
    }

    void SimulationTimeSource::Activate()
    {
        auto* registry = AZ::SettingsRegistry::Get();
        AZ_Assert(registry, "No Registry available");
        if (registry)
        {
            registry->Get(m_resetTimeOnRestart, ResetTimestampOnLevelReload);
        }

        auto* systemInterface = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (!systemInterface)
        {
            AZ_Warning("SimulationTimeSource", false, "Failed to get AzPhysics::SystemInterface");
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
                    AZ_Printf("SimulationTimeSource", "Registering clock to default scene");
                    if (m_resetTimeOnRestart)
                    {
                        AZ_Printf("SimulationTimeSource", "Reseting clock");
                        m_elapsed = 0.0;
                    }
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
                    AZ_Printf("SimulationTimeSource", "Removing clock to default scene");
                    m_onSceneSimulationEvent.Disconnect();
                }
            });
        systemInterface->RegisterSceneRemovedEvent(m_onSceneRemoved);
    }

    void SimulationTimeSource::Deactivate()
    {
        m_onSceneSimulationEvent.Disconnect();
    };

    builtin_interfaces::msg::Time SimulationTimeSource::GetROSTimestamp() const
    {
        builtin_interfaces::msg::Time timeStamp;
        timeStamp.sec = static_cast<int32_t>(AZStd::floor(m_elapsed));
        timeStamp.nanosec = static_cast<uint32_t>((m_elapsed - timeStamp.sec) * 1e9);
        return timeStamp;
    }

    AZ::Outcome<void, AZStd::string> SimulationTimeSource::AdjustTime(const builtin_interfaces::msg::Time& time)
    {
        const double timeSec = static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1e-9;
        m_elapsed = timeSec;
        return AZ::Success();
    }
} // namespace ROS2
