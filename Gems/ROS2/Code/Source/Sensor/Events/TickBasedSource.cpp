/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    void TickBasedSource::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TickBasedSource>()->Version(1)->Field("Source Enabled", &TickBasedSource::m_sourceEnabled);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<TickBasedSource>("Tick Based Source", "Sensor event source based on system ticks")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &TickBasedSource::m_sourceEnabled,
                        "Source Enabled",
                        "Enable/disable event source");
            }
        }
    }

    void TickBasedSource::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void TickBasedSource::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void TickBasedSource::Configure(const SensorConfiguration& sensorConfiguration)
    {
        m_sourceEnabled = sensorConfiguration.m_publishingEnabled;
    }

    void TickBasedSource::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_sourceEnabled)
        {
            return;
        }

        m_sensorSourceEvent.Signal(deltaTime);
    }
} // namespace ROS2