/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntitiesServiceHandler.h"
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <Clients/CommonUtilities.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{
    AZStd::unordered_set<SimulationFeatureType> GetEntitiesServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::ENTITY_TAGS,
                                                            SimulationFeatures::ENTITY_BOUNDS_BOX,
                                                            SimulationFeatures::ENTITY_BOUNDS_CONVEX,
                                                            SimulationFeatures::ENTITY_CATEGORIES };
    }

    AZStd::optional<GetEntitiesServiceHandler::Response> GetEntitiesServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZ::Outcome<SimulationInterfaces::EntityNameList, SimulationInterfaces::FailedResult> outcome;

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;

        const auto getFilterResult = SimulationInterfaces::Utils::GetEntityFiltersFromRequest<Request>(request);
        if (!getFilterResult.IsSuccess())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = getFilterResult.GetError().c_str();
            return response;
        }
        const SimulationInterfaces::EntityFilters filter = getFilterResult.GetValue();
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntities, filter);
        std::vector<std::string>& stdEntities = response.entities;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = failedResult.m_errorCode;
            response.result.error_message = failedResult.m_errorString.c_str();
            return response;
        }

        const auto& entityNameList = outcome.GetValue();
        AZStd::ranges::transform(entityNameList, AZStd::back_inserter(stdEntities), &AZStd::string::c_str);
        response.entities = stdEntities;

        return response;
    }
} // namespace ROS2SimulationInterfaces
