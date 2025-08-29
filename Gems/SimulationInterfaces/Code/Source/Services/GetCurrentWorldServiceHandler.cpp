/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetCurrentWorldServiceHandler.h"
#include <AzCore/std/string/string.h>
#include <Clients/CommonUtilities.h>
#include <SimulationInterfaces/LevelManagerRequestBus.h>
#include <SimulationInterfaces/WorldResource.h>
#include <simulation_interfaces/msg/result.hpp>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> GetCurrentWorldServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::WORLD_INFO_GETTING };
    }

    AZStd::optional<GetCurrentWorldServiceHandler::Response> GetCurrentWorldServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZ::Outcome<SimulationInterfaces::WorldResource, SimulationInterfaces::FailedResult> currentWorld;

        SimulationInterfaces::LevelManagerRequestBus::BroadcastResult(
            currentWorld, &SimulationInterfaces::LevelManagerRequests::GetCurrentWorld);
        Response response;
        if (!currentWorld.IsSuccess())
        {
            response.result.result = currentWorld.GetError().m_errorCode;
            response.result.error_message = currentWorld.GetError().m_errorString.c_str();
        }
        else
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            response.world = SimulationInterfaces::Utils::ConvertToRos2WorldResource(currentWorld.GetValue());
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
