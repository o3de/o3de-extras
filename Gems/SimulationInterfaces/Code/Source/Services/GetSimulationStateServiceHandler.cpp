/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetSimulationStateServiceHandler.h"
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/msg/simulation_state.hpp>

namespace ROS2SimulationInterfaces
{
    AZStd::unordered_set<AZ::u8> GetSimulationStateServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::SIMULATION_STATE_GETTING };
    }

    AZStd::optional<GetSimulationStateServiceHandler::Response> GetSimulationStateServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        Response response;
        SimulationInterfaces::SimulationState currentState;
        SimulationInterfaces::SimulationManagerRequestBus::BroadcastResult(
            currentState, &SimulationInterfaces::SimulationManagerRequests::GetSimulationState);
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        response.state.state = currentState;
        return response;
    }
} // namespace ROS2SimulationInterfaces
