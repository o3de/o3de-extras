/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntityStateServiceHandler.h"
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<AZ::u8> GetEntityStateServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::ENTITY_STATE_GETTING };
    }

    AZStd::optional<GetEntityStateServiceHandler::Response> GetEntityStateServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZStd::string entityName = request.entity.c_str();
        AZ::Outcome<SimulationInterfaces::EntityState, SimulationInterfaces::FailedResult> outcome;

        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntityState, entityName);

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
            response.result.error_message = failedResult.error_string.c_str();
            return response;
        }

        const auto& entityState = outcome.GetValue();
        simulation_interfaces::msg::EntityState entityStateMsg;
        entityStateMsg.header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        entityStateMsg.header.frame_id = ROS2::ROS2Interface::Get()->GetNode()->get_name();
        entityStateMsg.pose = ROS2::ROS2Conversions::ToROS2Pose(entityState.m_pose);
        entityStateMsg.twist.linear = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twist_linear);
        entityStateMsg.twist.angular = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twist_angular);

        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        response.state = entityStateMsg;
        return response;
    }

} // namespace ROS2SimulationInterfaces
