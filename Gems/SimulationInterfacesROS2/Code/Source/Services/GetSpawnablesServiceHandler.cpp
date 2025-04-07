/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetSpawnablesServiceHandler.h"
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{

    AZStd::unordered_set<AZ::u8> GetSpawnablesServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::SPAWNABLES };
    }

    AZStd::optional<GetSpawnablesServiceHandler::Response> GetSpawnablesServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZ::Outcome<SimulationInterfaces::SpawnableList, SimulationInterfaces::FailedResult> outcome;
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetSpawnables);
        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
            response.result.error_message = failedResult.error_string.c_str();
            return response;
        }

        const auto& spawnableList = outcome.GetValue();
        std::vector<simulation_interfaces::msg::Spawnable> simSpawnables;
        AZStd::transform(
            spawnableList.begin(),
            spawnableList.end(),
            AZStd::back_inserter(simSpawnables),
            [](const SimulationInterfaces::Spawnable& spawnable)
            {
                simulation_interfaces::msg::Spawnable simSpawnable;
                simSpawnable.uri = spawnable.m_uri.c_str();
                simSpawnable.description = spawnable.m_description.c_str();
                return simSpawnable;
            });
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        response.spawnables = simSpawnables;

        return response;
    }
} // namespace SimulationInterfacesROS2
