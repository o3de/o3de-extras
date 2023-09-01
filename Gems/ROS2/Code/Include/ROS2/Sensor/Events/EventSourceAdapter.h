/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/Events/SensorEventSource.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    namespace Internal
    {
        template<
            template<
                template<typename...>
                class, // EventType
                template<typename...>
                class, // EventHandlerType
                typename...> // Event parameters
            class C,
            class T>
        struct is_specialization_of : AZStd::false_type
        {
        };

        template<
            template<
                template<typename...>
                class, // EventType
                template<typename...>
                class, // EventHandlerType
                typename...> // Event parameters
            class Base,
            template<typename...>
            class EventType,
            template<typename...>
            class EventHandlerType,
            typename... Args>
        struct is_specialization_of<Base, Base<EventType, EventHandlerType, Args...>> : AZStd::true_type
        {
        };
    } // namespace Internal

    //! Class adapting event source (ROS2::SensorEventSource) to configurable working frequency. This is handled via adapted event, in
    //! a similar manner like it is done in SensorEventSource. EventSourceAdapter has its internal handler that connects to
    //! SensorEventSource source event, and signals adapted event according to frequency set (ROS2::EventSourceAdapter::Configure).
    //! User can connect to this event using ROS2::EventSourceAdapter::ConnectToAdaptedEvent method. This class should be used, instead
    //! of using directly a class derived from SensorEventSource, when specific working frequency is required. Following this path, user can
    //! still use source event - ROS2::EventSourceAdapter::ConnectToSourceEvent. This template has to be resolved using a class derived from
    //! SensorEventSource.
    //! @see ROS2::SensorEventSource
    template<class EventSourceT>
    class EventSourceAdapter
    {
    public:
        // Require non-abstract type derived from SensorEventSource.
        static_assert(Internal::is_specialization_of<SensorEventSource, typename EventSourceT::SourceBaseType>::value);
        static_assert(AZStd::is_base_of<typename EventSourceT::SourceBaseType, EventSourceT>::value);
        static_assert(AZStd::is_abstract<EventSourceT>::value == false);

        using SourceEventType = typename EventSourceT::SourceEventType;
        using SourceEventHandlerType = typename EventSourceT::SourceEventHandlerType;

        using AdaptedEventType = typename EventSourceT::AdaptedEventType;
        using AdaptedEventHandlerType = typename EventSourceT::AdaptedEventHandlerType;

        static void Reflect(AZ::ReflectContext* context)
        {
            EventSourceT::Reflect(context);

            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<EventSourceAdapter<EventSourceT>>()
                    ->Version(1)
                    ->Field("Adapted frequency", &EventSourceAdapter<EventSourceT>::m_adaptedFrequency)
                    ->Field("Event source configuration", &EventSourceAdapter<EventSourceT>::m_eventSource);

                if (auto editContext = serializeContext->GetEditContext())
                {
                    editContext
                        ->Class<EventSourceAdapter<EventSourceT>>(
                            "Event Source Adapter", "Adapts sensor event source to specified working frequency")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->ElementAttribute(AZ::Edit::Attributes::AutoExpand, true)
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &EventSourceAdapter<EventSourceT>::m_adaptedFrequency,
                            "Adapted frequency",
                            "Adapter event signalling frequency")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &EventSourceAdapter<EventSourceT>::m_eventSource,
                            "Event source configuration",
                            "Event source configuration")
                        ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
                }
            }
        }

        //! Activates event source adapter - assigns internal adapted event handler and activates managed event source. In most cases, user
        //! should call ROS2::EventSourceAdapter::Configure first - this adapter itself would work without this requirement, however
        //! different event source implementations can behave differently.
        void Activate()
        {
            m_sourceAdaptingEventHandler = typename EventSourceT::SourceEventHandlerType(
                [this](auto&&... args)
                {
                    const float sourceDeltaTime = m_eventSource.GetDeltaTime(args...);
                    m_adaptedDeltaTime += sourceDeltaTime;

                    if (!IsPublicationDeadline(sourceDeltaTime))
                    {
                        return;
                    }

                    m_sensorAdaptedEvent.Signal(m_adaptedDeltaTime, AZStd::forward<decltype(args)>(args)...);
                    m_adaptedDeltaTime = 0.0f;
                });
            m_eventSource.ConnectToSourceEvent(m_sourceAdaptingEventHandler);
            m_eventSource.Activate();
        }

        //! Deactivates event source adapter - deactivates event source and disconnects internal adapted event handler from source event. If
        //! it will be necessary, this implementation allows multiple consecutive calls of Activate / Deactivate, however user must also
        //! investigate specific event source implementation with such case in mind.
        void Deactivate()
        {
            m_eventSource.Deactivate();
            m_sourceAdaptingEventHandler.Disconnect();
        }

        //! Parses sensor configuration into event source configuration.
        //! @param sensorConfiguration Configuration of sensor using this event source adapter.
        void Configure(const SensorConfiguration& sensorConfiguration)
        {
            m_eventSource.Configure(sensorConfiguration);
            m_adaptedFrequency = sensorConfiguration.m_frequency;
        }

        //! Connects given event handler to source event (ROS2::SensorEventSource). That event is signalled regardless of adapted frequency
        //! set for event source adapter (ROS2::EventSourceAdapter::Configure). Its frequency depends only on specific event source
        //! implementation. If specific working frequency is required (main purpose of ROS2::EventSourceAdapter), user should see
        //! ROS2::EventSourceAdapter::ConnectToAdaptedEvent method.
        //! @param sourceEventHandler Event handler for source event (frequency not managed by event source adapter).
        //! @see ROS2::SensorEventSource
        void ConnectToSourceEvent(typename EventSourceT::SourceEventHandlerType& sourceEventHandler)
        {
            m_eventSource.ConnectToSourceEvent(sourceEventHandler);
        }

        //! Connects given event handler to adapted event (ROS2::EventSourceAdapter). This event is signalled with a frequency set in event
        //! source adapter configuration (ROS2::EventSourceAdapter::Configure).
        //! @param adaptedEventHandler Event handler for adapted event (frequency set through event source adapter configuration).
        void ConnectToAdaptedEvent(typename EventSourceT::AdaptedEventHandlerType& adaptedEventHandler)
        {
            adaptedEventHandler.Connect(m_sensorAdaptedEvent);
        }

    private:
        //! Uses tick counter, expected loop time and frequency set for adapter to support managing calls from event source.
        //! @param expectedLoopTime Expected simulation loop time.
        //! @return Whether it is time to signal adapted event.
        bool IsPublicationDeadline(float sourceDeltaTime)
        {
            if (--m_tickCounter > 0)
            {
                return false;
            }

            const float frameTime = m_adaptedFrequency == 0.f ? 1.f : 1.f / m_adaptedFrequency;
            const float numberOfFrames = frameTime / sourceDeltaTime;
            m_tickCounter = aznumeric_cast<int>(AZStd::round(numberOfFrames));
            return true;
        }

        EventSourceT m_eventSource{}; ///< Event source managed by this adapter.
        SourceEventHandlerType m_sourceAdaptingEventHandler{}; ///< Event handler for adapting event source to specific frequency.
        AdaptedEventType m_sensorAdaptedEvent{}; ///< Adapted event that is called with specific frequency.

        float m_adaptedFrequency{ 10.0f }; ///< Adapted frequency value.
        float m_adaptedDeltaTime{ 0.0f }; ///< Accumulator for calculating adapted delta time.
        int m_tickCounter{ 0 }; ///< Internal counter for controlling adapter frequency.
    };

    AZ_TYPE_INFO_TEMPLATE(EventSourceAdapter, "{DC8BB5F7-8E0E-42A1-BD82-5FCD9D31B9DD}", AZ_TYPE_INFO_CLASS)
} // namespace ROS2
