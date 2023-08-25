/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    //! Base class template for sensor event sources.
    //! Sample event sources can be based on TickBus or engine physics callback. Developer that wants to implement new event source, should
    //! derive it from this class and signal m_sensorEvent based on chosen approach. Derived classes should rather not auto connect any
    //! handler to m_sensorEvent, unless it is done to manage their internal logic. This event is for the final user that can connect
    //! prepared event handler to it using ConnectEventHandler method.
    //! @see ROS2::TickBasedSource.
    //! @see ROS2::PhysicsBasedSource.
    template<template<typename...> class EventT, template<typename...> class EventHandlerT, typename... EventArgs>
    class SensorEventSource
    {
    public:
        using SensorEventSourceType = SensorEventSource<EventT, EventHandlerT, EventArgs...>;
        using SensorEventType = EventT<EventArgs...>;
        using SensorEventHandlerType = EventHandlerT<EventArgs...>;

        AZ_TYPE_INFO(SensorEventSourceType, "{42ACE9CB-45ED-4BE9-9D5D-683436BE9FB2}");

        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<SensorEventSourceType>()->Version(1);
            }
        }

        virtual ~SensorEventSource() = default;

        //! Connects given event handler to sensor event. Event is signalled based on event source internal logic. For more details check
        //! event source specializations.
        //! @see ROS2::TickBasedSource.
        //! @see ROS2::PhysicsBasedSource.
        virtual void ConnectEventHandler(SensorEventHandlerType& eventHandler)
        {
            eventHandler.Connect(m_sensorEvent);
        }

    protected:
        SensorEventSource() = default;

        SensorEventType m_sensorEvent;
    };
} // namespace ROS2