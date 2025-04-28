/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SetSimulationStateServiceHandler.h"
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/msg/simulation_state.hpp>

namespace ROS2SimulationInterfaces
{
    AZStd::unordered_set<SimulationFeatureType> SetSimulationStateServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::SIMULATION_STATE_SETTING,
                                                            SimulationFeatures::SIMULATION_STATE_PAUSE };
    }

    AZStd::optional<SetSimulationStateServiceHandler::Response> SetSimulationStateServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        Response response;
        AZ::Outcome<void, SimulationInterfaces::FailedResult> transitionResult;
        SimulationInterfaces::SimulationManagerRequestBus::BroadcastResult(
            transitionResult,
            &SimulationInterfaces::SimulationManagerRequests::SetSimulationState,
            SimulationInterfaces::SimulationState(request.state.state));
        if (transitionResult.IsSuccess())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        }
        else
        {
            response.result.result = transitionResult.GetError().m_errorCode;
            response.result.error_message = transitionResult.GetError().m_errorString.c_str();
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
