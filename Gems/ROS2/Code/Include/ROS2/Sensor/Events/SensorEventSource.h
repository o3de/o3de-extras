/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace ROS2
{
    struct SensorConfiguration;

    //! Base class template for sensor event sources (based e.g. on TickBus or engine physics callback). Developer that wants to implement
    //! new event source should derive from this class and signal m_sensorEvent based on chosen approach. Derived classes can be used
    //! directly as unconstrained source of frequency - e.g. TickBasedSource, when synchronization with draw calls is necessary. However, in
    //! most common case they will be used as source of events for sensors, through EventSourceAdapter object, where their m_sensorEvent
    //! will be additionally adapted to some sensor-specific frequency.
    //! @see ROS2::EventSourceAdapter
    //! @see ROS2::TickBasedSource
    //! @see ROS2::PhysicsBasedSource
    template<template<typename...> class EventT, template<typename...> class EventHandlerT, typename... EventArgs>
    class SensorEventSource
    {
    public:
        using SourceBaseType = SensorEventSource<EventT, EventHandlerT, EventArgs...>;

        using SourceCallbackType = AZStd::function<void(EventArgs...)>;
        using AdaptedCallbackType = AZStd::function<void(float, EventArgs...)>;

        using SourceEventType = EventT<EventArgs...>;
        using SourceEventHandlerType = EventHandlerT<EventArgs...>;

        using AdaptedEventType = EventT<float, EventArgs...>;
        using AdaptedEventHandlerType = EventHandlerT<float, EventArgs...>;

        virtual ~SensorEventSource() = default;

        //! Starts event source - see specific event source description for more details. After call to this method event source is supposed
        //! to start signalling source event.
        virtual void Start()
        {
        }

        //! Stops event source - see specific event source description for more details. After call to this method event source is supposed
        //! to stop signalling source event.
        virtual void Stop()
        {
        }

        //! Returns delta time from set of source event parameters (passed as variadic up to some point). The purpose of this method is to
        //! give developers access to event source delta time, without additional variables or helpers.
        //! @param args Set of source event parameters.
        //! @return Delta time in seconds.
        [[nodiscard]] virtual float GetDeltaTime(EventArgs... args) const = 0;

        //! Connects given event handler to sensor source event. By design, source event is not restricted by frequency settings. This event
        //! should be signalled based on event source internal logic (e.g. on draw call). For more details check event source
        //! specializations.
        //! @param sourceEventHandler Event handler that will be connected to source event.
        void ConnectToSourceEvent(SourceEventHandlerType& sourceEventHandler)
        {
            sourceEventHandler.Connect(m_sourceEvent);
        }

    protected:
        SensorEventSource() = default;

        SourceEventType m_sourceEvent{}; ///< This event should be signalled based on specific event source specialization.
    };
} // namespace ROS2
