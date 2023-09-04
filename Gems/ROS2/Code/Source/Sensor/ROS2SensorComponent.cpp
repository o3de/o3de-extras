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
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void ROS2SensorComponent::Activate()
    {
        SetupRefreshLoop();
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2SensorComponent::Deactivate()
    {
        m_onTickCall.clear();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ROS2SensorComponent::Reflect(AZ::ReflectContext* context)
    {
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
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2SensorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        Visualize(); // each frame
        if (m_onTickCall)
        {
            m_onTickCall();
        }
    }

    bool ROS2SensorComponent::IsPublicationDeadline(float expectedLoopTime)
    {
        if (!m_sensorConfiguration.m_publishingEnabled)
        {
            return false;
        }
        m_tickCountDown--;
        if (m_tickCountDown <= 0)
        {
            const auto frequency = m_sensorConfiguration.m_frequency;
            const auto frameTime = frequency == 0.f ? 1.f : 1.f / frequency;
            const float numberOfFrames = frameTime / expectedLoopTime;
            m_tickCountDown = AZStd::round(numberOfFrames);
            return true;
        }
        return false;
    }

    void ROS2SensorComponent::SetupRefreshLoop()
    {
        m_onTickCall = [this]()
        {
            const AZStd::chrono::duration<float, AZStd::chrono::seconds::period> expectedLoopTime =
                ROS2Interface::Get()->GetSimulationClock().GetExpectedSimulationLoopTime();
            if (IsPublicationDeadline(expectedLoopTime.count()))
            {
                FrequencyTick();
            }
        };
    }

} // namespace ROS2
