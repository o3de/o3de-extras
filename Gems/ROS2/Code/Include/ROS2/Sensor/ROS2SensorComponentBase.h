/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Sensor/Events/EventSourceAdapter.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/Sensor/SensorConfigurationRequestBus.h>

namespace ROS2
{
    //! Base sensor component class for all specific sensor implementations. Developer working on the new sensor should derive from this
    //! class, defining necessary event source type (EventSourceT template parameter). Available sources are e.g. TickBasedSource or
    //! PhysicsBasedSource. Chosen event source is wrapped into EventSourceAdapter, making it possible to work with specific frequency.
    //! Derived implementation should call ROS2::ROS2SensorComponentBase::StartSensor at the end of Activate (or whenever sensor
    //! configuration is already set up) and StopSensor in Deactivate. Starting sensor base requires passing two parameters:
    //!  - sensor working frequency - how often sensor logic should be processed,
    //!  - adapted event callback - what should be done in sensor logic processing.
    //! Optionally, user can pass third parameter, which is source event callback - this will be called with source event frequency (check
    //! chosen event source implementation).
    //! @see ROS2::TickBasedSource
    //! @see ROS2::PhysicsBasedSource
    template<class EventSourceT>
    class ROS2SensorComponentBase
        : public AZ::Component
        , public SensorConfigurationRequestBus::Handler
    {
    public:
        using SensorBaseType = ROS2SensorComponentBase<EventSourceT>;

        AZ_COMPONENT_DECL((ROS2SensorComponentBase, AZ_CLASS));

        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<ROS2SensorComponentBase<EventSourceT>, AZ::Component>()->Version(1)->Field(
                    "SensorConfiguration", &ROS2SensorComponentBase<EventSourceT>::m_sensorConfiguration);

                if (auto* editContext = serializeContext->GetEditContext())
                {
                    editContext->Class<ROS2SensorComponentBase<EventSourceT>>("ROS2 Sensor Component Base", "Base component for sensors")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &ROS2SensorComponentBase<EventSourceT>::m_sensorConfiguration,
                            "Sensor configuration",
                            "Sensor configuration");
                }
            }
        }

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
        {
            required.push_back(AZ_CRC_CE("ROS2Frame"));
        }

        SensorConfiguration GetSensorConfiguration() const override
        {
            return m_sensorConfiguration;
        }

        void EnablePublishing(bool publishingEnabled) override
        {
            m_sensorConfiguration.m_publishingEnabled = publishingEnabled;
        }

        virtual ~ROS2SensorComponentBase() = default;

        void Activate() override
        {
        }

        void Deactivate() override
        {
        }

    protected:
        //! Starts sensor with passed frequency and adapted event callback. Optionally, user can pass source event callback, that will be
        //! called with event source frequency.
        //! @param sensorFrequency Sensor working frequency.
        //! @param adaptedCallback Adapted event callback - called with sensor working frequency.
        //! @param sourceCallback Source event callback - called with event source frequency.
        void StartSensor(
            float sensorFrequency,
            typename EventSourceT::AdaptedCallbackType adaptedCallback,
            typename EventSourceT::SourceCallbackType sourceCallback = nullptr)
        {
            m_eventSourceAdapter.SetFrequency(sensorFrequency);

            m_adaptedEventHandler.Disconnect();
            m_adaptedEventHandler = decltype(m_adaptedEventHandler)(adaptedCallback);
            m_eventSourceAdapter.ConnectToAdaptedEvent(m_adaptedEventHandler);

            m_sourceEventHandler.Disconnect();
            if (sourceCallback)
            {
                m_sourceEventHandler = decltype(m_sourceEventHandler)(sourceCallback);
                m_eventSourceAdapter.ConnectToSourceEvent(m_sourceEventHandler);
            }

            m_eventSourceAdapter.Start();
        }

        //! Stops sensor and disconnects event callbacks passed through RSO2::ROS2SensorComponentBase::StartSensor.
        void StopSensor()
        {
            m_eventSourceAdapter.Stop();
            m_sourceEventHandler.Disconnect();
            m_adaptedEventHandler.Disconnect();
        }

        //! Returns a complete namespace for this sensor topics and frame ids.
        [[nodiscard]] AZStd::string GetNamespace() const
        {
            auto* ros2Frame = GetEntity()->template FindComponent<ROS2FrameComponent>();

            return ros2Frame->GetNamespace();
        }

        //! Returns this sensor frame ID. The ID contains namespace.
        [[nodiscard]] AZStd::string GetFrameID() const
        {
            auto* ros2Frame = GetEntity()->template FindComponent<ROS2FrameComponent>();
            return ros2Frame->GetFrameID();
        }

        SensorConfiguration m_sensorConfiguration; ///< Basic sensor configuration.
        EventSourceAdapter<EventSourceT> m_eventSourceAdapter; ///< Adapter for selected event source (see this class documentation).

        //! Handler for source event. Requires manual assignment and connecting to source event in derived class.
        typename EventSourceT::SourceEventHandlerType m_sourceEventHandler;

        //! Handler for adapted event. Requires manual assignment and connecting to adapted event in derived class.
        typename EventSourceT::AdaptedEventHandlerType m_adaptedEventHandler;
    };

    AZ_COMPONENT_IMPL_INLINE(
        (ROS2SensorComponentBase, AZ_CLASS), "ROS2SensorComponentBase", "{2DF9A652-DF5D-43B1-932F-B6A838E36E97}", AZ::Component)
} // namespace ROS2
