/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "UnloadWorldServiceHandler.h"
#include <AzCore/std/string/string.h>
#include <SimulationInterfaces/LevelManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> UnloadWorldServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::WORLD_UNLOADING };
    }

    AZStd::optional<UnloadWorldServiceHandler::Response> UnloadWorldServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZ::Outcome<void, SimulationInterfaces::FailedResult> unloadWorld;

        SimulationInterfaces::LevelManagerRequestBus::BroadcastResult(
            unloadWorld, &SimulationInterfaces::LevelManagerRequests::UnloadWorld);
        Response response;
        if (!unloadWorld.IsSuccess())
        {
            response.result.result = unloadWorld.GetError().m_errorCode;
            response.result.error_message = unloadWorld.GetError().m_errorString.c_str();
        }
        else
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
