/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleDynamics/VehicleInputControlBus.h"
#include <StartingPointInput/InputEventNotificationBus.h>

// TODO - plenty of boilerplate code, seems somewhat redundant since it would be better to be able to map inputs directly
namespace VehicleDynamics
{
    //! A handler for a single input event.
    class ManualControlSingleEventHandler : private StartingPointInput::InputEventNotificationBus::Handler
    {
    public:
        using OnHeldHandlerFunction = std::function<void(float)>;

        //! Construct the event handler.
        //! @param eventName which event to handle (e.g. "steering", "accelerate")
        //! @param handler a function which handles the input, typically through re-publishing it to a vehicle input bus.
        ManualControlSingleEventHandler(const AZStd::string& eventName, OnHeldHandlerFunction handler)
            : m_eventName(eventName)
            , m_handler(handler)
        {
        }

        void Activate()
        {
            StartingPointInput::InputEventNotificationBus::Handler::BusConnect(
                StartingPointInput::InputEventNotificationId(m_eventName.c_str()));
        }

        void Deactivate()
        {
            StartingPointInput::InputEventNotificationBus::Handler::BusDisconnect(
                StartingPointInput::InputEventNotificationId(m_eventName.c_str()));
        }

    private:
        AZStd::string m_eventName;
        OnHeldHandlerFunction m_handler;

        void OnHeld(float value) override
        {
            m_handler(value);
        }
    };

    //! Registers to "steering" and "acceleration" input events, and translates them into vehicle inputs
    class ManualControlEventHandler
    {
    public:
        ManualControlEventHandler()
        {
            m_eventHandlers.push_back(ManualControlSingleEventHandler(
                "steering",
                [](float inputValue)
                {
                    VehicleInputControlRequestBus::Broadcast(&VehicleInputControlRequests::SetTargetSteeringFraction, inputValue);
                }));

            m_eventHandlers.push_back(ManualControlSingleEventHandler(
                "accelerate",
                [](float inputValue)
                {
                    VehicleInputControlRequestBus::Broadcast(&VehicleInputControlRequests::SetTargetLinearSpeedFraction, inputValue);
                }));
        }

        void Activate()
        {
            for (auto& handler : m_eventHandlers)
            {
                handler.Activate();
            }
        }

        void Deactivate()
        {
            for (auto& handler : m_eventHandlers)
            {
                handler.Deactivate();
            }
        }

    private:
        AZStd::vector<ManualControlSingleEventHandler> m_eventHandlers;
    };
} // namespace VehicleDynamics