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
    //! new event source, should derive it from this class and signal m_sensorEvent based on chosen approach. Derived classes should rather
    //! not auto connect any handler to m_sensorEvent, unless it is done to manage their internal logic. This event is for the final user
    //! that can connect prepared event handler to it using ConnectToSourceEvent method.
    //! @see ROS2::TickBasedSource.
    //! @see ROS2::PhysicsBasedSource.
    template<template<typename...> class EventT, template<typename...> class EventHandlerT, typename... EventArgs>
    class SensorEventSource
    {
    public:
        using SensorEventSourceType = SensorEventSource<EventT, EventHandlerT, EventArgs...>;
        using SensorEventType = EventT<EventArgs...>;
        using SensorEventHandlerType = EventHandlerT<EventArgs...>;

        virtual ~SensorEventSource() = default;

        //! Sets up event source - ee event source description for more details. After call to this method event source is supposed to start
        //! signalling source event.
        virtual void Activate()
        {
        }

        //! Shuts down event source - see event source description for more details. After call to this method event source is supposed to
        //! stop signalling source event.
        virtual void Deactivate()
        {
        }

        //! Configures event source based on sensor settings. Details depend on specific event source implementation.
        //! @param sensorConfiguration Implemented sensor configuration.
        //! @note In most cases it will be safer to call this before Activate, however at the end it depends on specific event source
        //! implementations.
        virtual void Configure([[maybe_unused]] const SensorConfiguration& sensorConfiguration)
        {
        }

        //! Connects given event handler to sensor source event. By design, source event is not restricted by frequency settings. This event
        //! should be signalled based on event source internal logic (e.g. on draw call). For more details check event source
        //! specializations.
        //! @param sourceEventHandler Event handler that will be connected to source event.
        void ConnectToSourceEvent(SensorEventHandlerType& sourceEventHandler)
        {
            sourceEventHandler.Connect(m_sensorSourceEvent);
        }

    protected:
        SensorEventSource() = default;

        SensorEventType m_sensorSourceEvent{}; ///< This event should be signalled based on specific event source specialization.
    };
} // namespace ROS2