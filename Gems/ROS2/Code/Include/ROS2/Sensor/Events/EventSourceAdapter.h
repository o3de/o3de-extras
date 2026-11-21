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
#include <ROS2/ROS2TypeIds.h>
#include <AzCore/std/optional.h>
#include <ROS2/Sensor/Events/SensorEventSource.h>
#include <ROS2/Sensor/Events/SensorPacing.h>

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
        //! set using ROS2::EventSourceAdapter::SetFrequency method. The pacing comes from the registry (ROS2::SensorPacingRegistryKey)
        //! unless ROS2::EventSourceAdapter::SetPacing set it; event sources with a SetPacing method get the same pacing.
        void Start()
        {
            m_pacing = m_pacingOverride.value_or(GetSensorPacingSetting());
            if constexpr (requires(EventSourceT& source, SensorPacing pacing) { source.SetPacing(pacing); })
            {
                m_eventSource.SetPacing(m_pacing);
            }
            m_timeSinceDeadline = 0.0f;
            m_sourceAdaptingEventHandler = typename EventSourceT::SourceEventHandlerType(
                [this](auto&&... args)
                {
                    const float sourceDeltaTime = m_eventSource.GetDeltaTime(AZStd::forward<decltype(args)>(args)...);
                    m_adaptedDeltaTime += sourceDeltaTime;

                    if (!IsPublicationDeadline(sourceDeltaTime))
                    {
                        return;
                    }

                    m_lastDeltaTime = m_adaptedDeltaTime;
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

        //! Sets how the adapted event is paced (see ROS2::SensorPacing), instead of the registry setting. Takes effect on Start.
        void SetPacing(SensorPacing pacing)
        {
            m_pacingOverride = pacing;
        }

        //! Gets adapter working frequency, based on the last obtained delta time between adapted events.
        //! If the adapter has not been started yet, the frequency will be zero.
        [[nodiscard]] float GetEffectiveFrequency() const
        {
            if (m_lastDeltaTime == 0.0f)
            {
                return 0.0f;
            }
            return 1.0f / m_lastDeltaTime;
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
        //! Decides whether the adapted event is due, according to the pacing (see ROS2::SensorPacing).
        //! @param sourceDeltaTime Delta time of event source.
        //! @return Whether it is time to signal adapted event.
        [[nodiscard]] bool IsPublicationDeadline(float sourceDeltaTime)
        {
            return m_pacing == SensorPacing::SimulationTime ? IsSimulationTimeDeadline(sourceDeltaTime)
                                                            : IsFrameCountDeadline(sourceDeltaTime);
        }

        //! SensorPacing::Frames. Uses:
        //!  - internal tick counter,
        //!  - last delta time of event source and
        //!  - frequency set for adapter
        //! to support managing calls from event source. In other words, uses delta time of event source to calculate average number of
        //! source event calls per adapted event call.
        [[nodiscard]] bool IsFrameCountDeadline(float sourceDeltaTime)
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

        //! SensorPacing::SimulationTime: accumulates the time since the last deadline, keeps the remainder past each deadline, and
        //! signals on the source event closest to it. After a stall longer than two periods, the backlog is dropped instead of
        //! signalling in a burst.
        [[nodiscard]] bool IsSimulationTimeDeadline(float sourceDeltaTime)
        {
            const float period = m_adaptedFrequency > 0.0f ? 1.0f / m_adaptedFrequency : 1.0f;
            m_timeSinceDeadline += sourceDeltaTime;
            if (m_timeSinceDeadline + 0.5f * sourceDeltaTime < period)
            {
                return false;
            }
            m_timeSinceDeadline = m_timeSinceDeadline < 2.0f * period ? m_timeSinceDeadline - period : 0.0f;
            return true;
        }

        EventSourceT m_eventSource{}; ///< Event source managed by this adapter.

        //! Event handler for adapting event source to specific frequency.
        typename EventSourceT::SourceEventHandlerType m_sourceAdaptingEventHandler{};

        //! Adapted event that is called with specific frequency.
        typename EventSourceT::AdaptedEventType m_sensorAdaptedEvent{};

        float m_adaptedFrequency{ 30.0f }; ///< Adapted frequency value.
        float m_lastDeltaTime{ 0.0f }; ///< Last difference in time between adapted events, used to compute effective frequency value.
        float m_adaptedDeltaTime{ 0.0f }; ///< Accumulator for calculating adapted delta time.
        SensorPacing m_pacing{ SensorPacing::Frames }; ///< Pacing in use, set on Start.
        AZStd::optional<SensorPacing> m_pacingOverride; ///< Pacing set with SetPacing, which replaces the registry setting.
        int m_tickCounter{ 0 }; ///< SensorPacing::Frames: source events until the next adapted event.
        float m_timeSinceDeadline{ 0.0f }; ///< SensorPacing::SimulationTime: source time since the last deadline.
    };

    AZ_TYPE_INFO_TEMPLATE(EventSourceAdapter, EventSourceAdapterTypeId, AZ_TYPE_INFO_CLASS)
} // namespace ROS2
