/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SetEntityInfoServiceHandler.h"
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> SetEntityInfoServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::ENTITY_INFO_SETTING };
    }

    AZStd::optional<SetEntityInfoServiceHandler::Response> SetEntityInfoServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZStd::string entityName = request.entity.c_str();
        SimulationInterfaces::EntityInfo entityInfo;
        entityInfo.m_category = request.info.category.category;
        entityInfo.m_description = request.info.description.c_str();
        AZStd::ranges::transform(request.info.tags, AZStd::back_inserter(entityInfo.m_tags), &std::string::c_str);

        AZ::Outcome<void, SimulationInterfaces::FailedResult> outcome;
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::SetEntityInfo, entityName, entityInfo);

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = failedResult.m_errorCode;
            response.result.error_message = failedResult.m_errorString.c_str();
            return response;
        }

        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        return response;
    }

} // namespace ROS2SimulationInterfaces
