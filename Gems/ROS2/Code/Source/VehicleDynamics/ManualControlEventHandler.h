/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/function/function_template.h>
#include <ROS2/VehicleDynamics/VehicleInputControlBus.h>
#include <StartingPointInput/InputEventNotificationBus.h>

namespace ROS2::VehicleDynamics
{
    //! A handler for a single input event.
    class ManualControlSingleEventHandler : private StartingPointInput::InputEventNotificationBus::Handler
    {
    public:
        using OnHeldHandlerFunction = AZStd::function<void(float)>;

        //! Construct the event handler.
        //! @param eventName which event to handle (eg "steering", "accelerate")
        //! @param handler a function which handles the input, typically through re-publishing it to a vehicle input bus.
        ManualControlSingleEventHandler(const AZStd::string& eventName, OnHeldHandlerFunction handler)
            : m_eventName(eventName)
            , m_handler(AZStd::move(handler))
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

        void OnPressed(float value) override
        {
            m_handler(value);
        }

        void OnHeld(float value) override
        {
            m_handler(value);
        }

        void OnReleased([[maybe_unused]] float value) override
        {
            m_handler(0);
        }
    };

    //! Registers to "steering" and "acceleration" input events, and translates them into vehicle inputs
    class ManualControlEventHandler
    {
    public:
        void Activate(AZ::EntityId ownerEntityId)
        {
            m_eventHandlers.push_back(ManualControlSingleEventHandler(
                "steering",
                [ownerEntityId](float inputValue)
                {
                    VehicleInputControlRequestBus::Event(
                        ownerEntityId, &VehicleInputControlRequests::SetTargetSteeringFraction, inputValue);
                }));

            m_eventHandlers.push_back(ManualControlSingleEventHandler(
                "accelerate",
                [ownerEntityId](float inputValue)
                {
                    VehicleInputControlRequestBus::Event(
                        ownerEntityId, &VehicleInputControlRequests::SetTargetLinearSpeedFraction, inputValue);
                }));

            m_eventHandlers.push_back(ManualControlSingleEventHandler(
                "rotate",
                [ownerEntityId](float inputValue)
                {
                    VehicleInputControlRequestBus::Event(
                        ownerEntityId, &VehicleInputControlRequests::SetTargetAngularSpeedFraction, inputValue);
                }));

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
} // namespace ROS2::VehicleDynamics