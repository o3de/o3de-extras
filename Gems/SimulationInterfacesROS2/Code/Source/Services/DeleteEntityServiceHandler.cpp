/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "DeleteEntityServiceHandler.h"
#include <AzCore/std/string/string.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{

    AZStd::unordered_set<AZ::u8> DeleteEntityServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::DELETING };
    }

    DeleteEntityServiceHandler::Response DeleteEntityServiceHandler::HandleServiceRequest(
        const rmw_request_id_t& header, const Request& request)
    {
        AZStd::string entityName = request.entity.c_str();
        Response response;
        SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationEntityManagerRequests::DeleteEntity,
            entityName,
            [&response](const AZ::Outcome<void, SimulationInterfaces::FailedResult>& outcome)
            {
                if (outcome.IsSuccess())
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                }
                else
                {
                    const auto& failedResult = outcome.GetError();
                    response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
                    response.result.error_message = failedResult.error_string.c_str();
                }
            });
        return response;
    }
} // namespace SimulationInterfacesROS2
