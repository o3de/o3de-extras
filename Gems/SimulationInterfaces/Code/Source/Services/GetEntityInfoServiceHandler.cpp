/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntityInfoServiceHandler.h"
#include "Utils/RegistryUtils.h"
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <simulation_interfaces/msg/detail/entity_info__struct.hpp>
namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> GetEntityInfoServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::ENTITY_INFO_GETTING };
    }

    AZStd::optional<GetEntityInfoServiceHandler::Response> GetEntityInfoServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZStd::string entityName = request.entity.c_str();
        AZ::Outcome<SimulationInterfaces::EntityInfo, SimulationInterfaces::FailedResult> outcome;

        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntityInfo, entityName);

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = failedResult.m_errorCode;
            response.result.error_message = failedResult.m_errorString.c_str();
            return response;
        }

        const auto simulatorFrameId = RegistryUtilities::GetSimulatorROS2Frame();
        simulation_interfaces::msg::EntityInfo entityInfoMsg;
        entityInfoMsg.category.category = outcome.GetValue().m_category;
        entityInfoMsg.description = outcome.GetValue().m_description.c_str();

        AZStd::transform(
            outcome.GetValue().m_tags.begin(),
            outcome.GetValue().m_tags.end(),
            AZStd::back_inserter(entityInfoMsg.tags),
            [](const AZStd::string& item)
            {
                return item.c_str();
            });

        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        response.info = entityInfoMsg;
        return response;
    }

} // namespace ROS2SimulationInterfaces
