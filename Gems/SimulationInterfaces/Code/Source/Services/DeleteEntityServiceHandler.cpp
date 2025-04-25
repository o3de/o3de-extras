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

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> DeleteEntityServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::DELETING };
    }

    AZStd::optional<DeleteEntityServiceHandler::Response> DeleteEntityServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZStd::string entityName = request.entity.c_str();

        SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationEntityManagerRequests::DeleteEntity,
            entityName,
            [this](const AZ::Outcome<void, SimulationInterfaces::FailedResult>& outcome)
            {
                Response response;
                if (outcome.IsSuccess())
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                }
                else
                {
                    const auto& failedResult = outcome.GetError();
                    response.result.result = failedResult.m_errorCode;
                    response.result.error_message = failedResult.m_errorString.c_str();
                }
                SendResponse(response);
            });
        return AZStd::nullopt;
    }
} // namespace ROS2SimulationInterfaces
