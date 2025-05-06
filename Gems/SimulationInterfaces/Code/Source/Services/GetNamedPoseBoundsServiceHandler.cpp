/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetNamedPoseBoundsServiceHandler.h"

#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/NamedPoseManagerRequestBus.h>
#include <SimulationInterfaces/Result.h>
#include <simulation_interfaces/msg/bounds.hpp>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> GetNamedPoseBoundsServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::POSE_BOUNDS };
    }

    AZStd::optional<GetNamedPoseBoundsServiceHandler::Response> GetNamedPoseBoundsServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        using namespace SimulationInterfaces;
        AZ::Outcome<Bounds, FailedResult> namedPoseBoundsO3DE;
        Response response;
        if (request.name.empty())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = "Passing empty name is forbidden";
            return response;
        }

        NamedPoseManagerRequestBus::BroadcastResult(
            namedPoseBoundsO3DE, &NamedPoseManagerRequests::GetNamedPoseBounds, AZStd::string{ request.name.c_str() });

        if (!namedPoseBoundsO3DE.IsSuccess())
        {
            response.result.result = namedPoseBoundsO3DE.GetError().m_errorCode;
            response.result.error_message = namedPoseBoundsO3DE.GetError().m_errorString.c_str();
        }
        else
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            response.bounds.type = namedPoseBoundsO3DE.GetValue().m_boundsType;
            for (auto& point : namedPoseBoundsO3DE.GetValue().m_points)
            {
                response.bounds.points.emplace_back(ROS2::ROS2Conversions::ToROS2Vector3(point));
            }
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
