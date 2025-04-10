/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "StepSimulationServiceHandler.h"
#include <AzCore/std/string/string.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<AZ::u8> StepSimulationServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::STEP_SIMULATION_SINGLE,
                                             SimulationFeatures::STEP_SIMULATION_MULTIPLE };
    }

    AZStd::optional<StepSimulationServiceHandler::Response> StepSimulationServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        bool isPaused = false;
        SimulationInterfaces::SimulationManagerRequestBus::BroadcastResult(
            isPaused, &SimulationInterfaces::SimulationManagerRequests::IsSimulationPaused);
        if (!isPaused)
        {
            Response response;
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = "Request cannot be processed - simulation has to be paused.";
            return response;
        }

        SimulationInterfaces::SimulationManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationManagerRequests::StepSimulation, request.steps);
        AZ::TickBus::Handler::BusConnect();
        return AZStd::nullopt;
    }

    void StepSimulationServiceHandler::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        bool isActive = true;
        SimulationInterfaces::SimulationManagerRequestBus::BroadcastResult(
            isActive, &SimulationInterfaces::SimulationManagerRequests::IsSimulationStepsActive);
        if (!isActive)
        {
            Response response;
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            SendResponse(response);
            AZ::TickBus::Handler::BusDisconnect();
        }
    }
} // namespace ROS2SimulationInterfaces
