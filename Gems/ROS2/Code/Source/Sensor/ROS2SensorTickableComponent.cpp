/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Sensor/ROS2SensorTickableComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void ROS2SensorTickableComponent::Activate()
    {
        ROS2SensorComponent::Activate();
    }

    void ROS2SensorTickableComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
    }

    void ROS2SensorTickableComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SensorTickableComponent, ROS2SensorComponent>()->Version(1);
        }
    }

    void ROS2SensorTickableComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        ROS2SensorComponent::OnTick(deltaTime,time);
        const AZStd::chrono::duration<float, AZStd::chrono::seconds::period> expectedLoopTime =
            ROS2Interface::Get()->GetSimulationClock().GetExpectedSimulationLoopTime();
        if (IsPublicationDeadline(deltaTime, expectedLoopTime.count()))
        {
            FrequencyTick();
        }
    }

} // namespace ROS2
