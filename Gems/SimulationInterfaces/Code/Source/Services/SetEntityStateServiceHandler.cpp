/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SetEntityStateServiceHandler.h"
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> SetEntityStateServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::ENTITY_STATE_SETTING };
    }

    AZStd::optional<SetEntityStateServiceHandler::Response> SetEntityStateServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZ::Outcome<void, SimulationInterfaces::FailedResult> outcome;
        AZStd::string entityName = request.entity.c_str();
        SimulationInterfaces::EntityState entityState;
        entityState.m_pose = ROS2::ROS2Conversions::FromROS2Pose(request.state.pose);
        entityState.m_twistAngular = ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.angular);
        entityState.m_twistLinear = ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.linear);

        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::SetEntityState, entityName, entityState);

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = aznumeric_cast<uint8_t>(failedResult.m_errorCode);
            response.result.error_message = failedResult.m_errorString.c_str();
        }

        return response;
    }
} // namespace ROS2SimulationInterfaces
