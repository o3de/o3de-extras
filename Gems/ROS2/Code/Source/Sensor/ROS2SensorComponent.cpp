/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2/Sensor/ROS2SensorComponent.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "ROS2/ROS2GemUtilities.h"
#include "ROS2/Utilities/ROS2Names.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2SensorComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2SensorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ROS2SensorComponent::Reflect(AZ::ReflectContext* context)
    {
        SensorConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SensorComponent, AZ::Component>()->Version(1)->Field(
                "SensorConfiguration", &ROS2SensorComponent::m_sensorConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SensorComponent>("ROS2 Sensor", "Base component for sensors")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SensorComponent::m_sensorConfiguration,
                        "Sensor configuration",
                        "Sensor configuration");
            }
        }
    }

    AZStd::string ROS2SensorComponent::GetNamespace() const
    {
        // TODO - hold frame?
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetNamespace();
    };

    AZStd::string ROS2SensorComponent::GetFrameID() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetFrameID();
    }

    void ROS2SensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("ROS2Frame"));
    }

    void ROS2SensorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        Visualise(); // each frame
        if (!m_sensorConfiguration.m_publishingEnabled)
        {
            return;
        }

        auto frequency = m_sensorConfiguration.m_frequency;

        // TODO - add range validation (Attributes?)
        auto frameTime = frequency == 0 ? 1 : 1 / frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick < frameTime)
            return;

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }

        // Note that sensor frequency can be limited by simulation tick rate (if higher sensor Hz is desired).
        FrequencyTick();
    }
} // namespace ROS2
