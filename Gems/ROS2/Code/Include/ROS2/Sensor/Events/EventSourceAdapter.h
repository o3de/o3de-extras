/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Serialization/EditContext.h>
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
    //! SensorEventSource source event, and signals adapted event according to frequency set (ROS2::EventSourceAdapter::SetFrequency).
    //! User can connect to this event using ROS2::EventSourceAdapter::ConnectToAdaptedEvent method. This class should be used, instead
    //! of using directly a class derived from SensorEventSource, when specific working frequency is required. Following this path, user can
    //! still use source event - ROS2::EventSourceAdapter::ConnectToSourceEvent. This template has to be resolved using a class derived from
    //! SensorEventSource specialization.
    //! @see ROS2::SensorEventSource
    template<class EventSourceT>
    class EventSourceAdapter
    {
    public:
        // Require non-abstract type derived from SensorEventSource specialization.
        static_assert(Internal::is_specialization_of<SensorEventSource, typename EventSourceT::SourceBaseType>::value);
        static_assert(AZStd::is_base_of<typename EventSourceT::SourceBaseType, EventSourceT>::value);
        static_assert(AZStd::is_abstract<EventSourceT>::value == false);

        static void Reflect(AZ::ReflectContext* context)
        {
            EventSourceT::Reflect(context);

            if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<EventSourceAdapter<EventSourceT>>()
                    ->Version(1)
                    ->Field("Adapted frequency", &EventSourceAdapter<EventSourceT>::m_adaptedFrequency)
                    ->Field("Event source configuration", &EventSourceAdapter<EventSourceT>::m_eventSource);

                if (auto* editContext = serializeContext->GetEditContext())
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
                            "Adapter event signalling frequency");
                }
            }
        }

        //! Starts event source adapter - assigns internal adapted event handler and starts managed event source. Adapted frequency can be
        //! set using ROS2::EventSourceAdapter::SetFrequency method.
        void Start()
        {
            m_sourceAdaptingEventHandler = typename EventSourceT::SourceEventHandlerType(
                [this](auto&&... args)
                {
                    const float sourceDeltaTime = m_eventSource.GetDeltaTime(AZStd::forward<decltype(args)>(args)...);
                    m_adaptedDeltaTime += sourceDeltaTime;

                    if (!IsPublicationDeadline(sourceDeltaTime))
                    {
                        return;
                    }

                    m_sensorAdaptedEvent.Signal(m_adaptedDeltaTime, AZStd::forward<decltype(args)>(args)...);
                    m_adaptedDeltaTime = 0.0f;
                });
            m_eventSource.ConnectToSourceEvent(m_sourceAdaptingEventHandler);
            m_eventSource.Start();
        }

        //! Stops event source adapter - stops event source and disconnects internal adapted event handler from source event. If it will be
        //! necessary, this implementation allows multiple consecutive calls of Start / Stop, however user must also investigate specific
        //! event source implementation with such case in mind.
        void Stop()
        {
            m_eventSource.Stop();
            m_sourceAdaptingEventHandler.Disconnect();
        }

        //! Sets adapter working frequency. By design, adapter will not work correctly, if this frequency will be greater than used event
        //! source frequency - e.g. adapter will be requested to work in 60Hz, when using event source working in 30Hz. In general, adapted
        //! frequency should be equal or lower than event source frequency - this is forced internally
        //! (ROS2::EventSourceAdapter::IsPublicationDeadline). Optimal (highest precision in timing events) working conditions take place
        //! when event source frequency is an integer multiple of adapted frequency.
        //! @param adaptedFrequency Adapter working frequency. When set to zero or less adapter will be assumed to work in 1Hz.
        void SetFrequency(float adaptedFrequency)
        {
            m_adaptedFrequency = adaptedFrequency;
        }

        //! Connects given event handler to source event (ROS2::SensorEventSource). That event is signalled regardless of adapted frequency
        //! set for event source adapter (ROS2::EventSourceAdapter::SetFrequency). Its frequency depends only on specific event source
        //! implementation. If different working frequency is required (main purpose of ROS2::EventSourceAdapter), user should see
        //! ROS2::EventSourceAdapter::ConnectToAdaptedEvent method.
        //! @param sourceEventHandler Event handler for source event (frequency not managed by event source adapter).
        //! @see ROS2::SensorEventSource
        void ConnectToSourceEvent(typename EventSourceT::SourceEventHandlerType& sourceEventHandler)
        {
            m_eventSource.ConnectToSourceEvent(sourceEventHandler);
        }

        //! Connects given event handler to adapted event (ROS2::EventSourceAdapter). This event is signalled with a frequency set with
        //! ROS2::EventSourceAdapter::SetFrequency method.
        //! @param adaptedEventHandler Event handler for adapted event.
        void ConnectToAdaptedEvent(typename EventSourceT::AdaptedEventHandlerType& adaptedEventHandler)
        {
            adaptedEventHandler.Connect(m_sensorAdaptedEvent);
        }

    private:
        //! Uses:
        //!  - internal tick counter,
        //!  - last delta time of event source and
        //!  - frequency set for adapter
        //! to support managing calls from event source. In other words, uses delta time of event source to calculate average number of
        //! source event calls per adapted event call.
        //! @param sourceDeltaTime Delta time of event source.
        //! @return Whether it is time to signal adapted event.
        [[nodiscard]] bool IsPublicationDeadline(float sourceDeltaTime)
        {
            if (--m_tickCounter > 0)
            {
                return false;
            }

            const float sourceFrequencyEstimation = 1.0f / sourceDeltaTime;
            const float numberOfFrames =
                m_adaptedFrequency <= sourceFrequencyEstimation ? (sourceFrequencyEstimation / m_adaptedFrequency) : 1.0f;
            m_tickCounter = aznumeric_cast<int>(AZStd::round(numberOfFrames));
            return true;
        }

        EventSourceT m_eventSource{}; ///< Event source managed by this adapter.

        //! Event handler for adapting event source to specific frequency.
        typename EventSourceT::SourceEventHandlerType m_sourceAdaptingEventHandler{};

        //! Adapted event that is called with specific frequency.
        typename EventSourceT::AdaptedEventType m_sensorAdaptedEvent{};

        float m_adaptedFrequency{ 30.0f }; ///< Adapted frequency value.
        float m_adaptedDeltaTime{ 0.0f }; ///< Accumulator for calculating adapted delta time.
        int m_tickCounter{ 0 }; ///< Internal counter for controlling adapter frequency.
    };

    AZ_TYPE_INFO_TEMPLATE(EventSourceAdapter, "{DC8BB5F7-8E0E-42A1-BD82-5FCD9D31B9DD}", AZ_TYPE_INFO_CLASS)
} // namespace ROS2
