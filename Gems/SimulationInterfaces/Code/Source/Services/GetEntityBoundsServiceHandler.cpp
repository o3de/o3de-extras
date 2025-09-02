/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntityBoundsServiceHandler.h"

#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/Result.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <simulation_interfaces/msg/bounds.hpp>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> GetEntityBoundsServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::ENTITY_BOUNDS, SimulationFeatures::ENTITY_BOUNDS_BOX };
    }

    AZStd::optional<GetEntityBoundsServiceHandler::Response> GetEntityBoundsServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        using namespace SimulationInterfaces;
        AZ::Outcome<Bounds, FailedResult> entityBounds;
        Response response;
        if (request.entity.empty())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = "Passing empty name is forbidden";
            return response;
        }

        SimulationEntityManagerRequestBus::BroadcastResult(
            entityBounds, &SimulationEntityManagerRequests::GetEntityBounds, AZStd::string{ request.entity.c_str() });

        if (!entityBounds.IsSuccess())
        {
            response.result.result = entityBounds.GetError().m_errorCode;
            response.result.error_message = entityBounds.GetError().m_errorString.c_str();
        }
        else
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            response.bounds.type = entityBounds.GetValue().m_boundsType;
            for (const auto& point : entityBounds.GetValue().m_points)
            {
                response.bounds.points.emplace_back(ROS2::ROS2Conversions::ToROS2Vector3(point));
            }
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
