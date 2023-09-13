/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/OrderedEvent.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <ROS2/Sensor/Events/SensorEventSource.h>

namespace ROS2
{
    //! Class implementing physics callbacks as sensor event source. Source event (ROS2::SensorEventSource) is signalled based on scene
    //! simulation finish event.
    //! @note The working frequency of this event source can be changed through engine settings (physics simulation delta).
    //! @see ROS2::SensorEventSource
    class PhysicsBasedSource final : public SensorEventSource<AZ::OrderedEvent, AZ::OrderedEventHandler, AzPhysics::SceneHandle, float>
    {
    public:
        AZ_TYPE_INFO(PhysicsBasedSource, "{48BB21A8-F14E-4869-95DC-28EEA279Cf53}");
        static void Reflect(AZ::ReflectContext* context);

        // Overrides of ROS2::SensorEventSource.
        void Start() override;
        void Stop() override;
        [[nodiscard]] float GetDeltaTime(AzPhysics::SceneHandle sceneHandle, float deltaTime) const override;

    private:
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_onSceneSimulationEventHandler; ///< Handler for physics callback.
    };
} // namespace ROS2
