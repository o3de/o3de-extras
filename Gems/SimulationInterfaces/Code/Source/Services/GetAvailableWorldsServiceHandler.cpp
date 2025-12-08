/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetAvailableWorldsServiceHandler.h"
#include "SimulationInterfaces/TagFilter.h"
#include "SimulationInterfaces/WorldResource.h"
#include <AzCore/std/iterator.h>
#include <AzCore/std/ranges/ranges_algorithm.h>
#include <AzCore/std/string/string.h>
#include <Clients/CommonUtilities.h>
#include <SimulationInterfaces/LevelManagerRequestBus.h>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/msg/tags_filter.hpp>
#include <simulation_interfaces/msg/world_resource.hpp>

namespace ROS2SimulationInterfaces
{
    namespace
    {
        [[nodiscard]] SimulationInterfaces::GetWorldsRequest ConvertROS2Request(
            const GetAvailableWorldsServiceHandler::Request& ros2Request)
        {
            SimulationInterfaces::GetWorldsRequest request;
            request.continueOnError = ros2Request.continue_on_error;
            request.offlineOnly = ros2Request.offline_only;
            // copy additional sources
            AZStd::ranges::transform(ros2Request.additional_sources, AZStd::back_inserter(request.additionalSources), &std::string::c_str);
            request.filter.m_mode = ros2Request.filter.filter_mode;
            AZStd::ranges::transform(
                ros2Request.filter.tags, AZStd::inserter(request.filter.m_tags, request.filter.m_tags.end()), &std::string::c_str);
            return request;
        }
    } // namespace

    AZStd::unordered_set<SimulationFeatureType> GetAvailableWorldsServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::AVAILABLE_WORLDS };
    }

    AZStd::optional<GetAvailableWorldsServiceHandler::Response> GetAvailableWorldsServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        Response response;
        AZ::Outcome<SimulationInterfaces::WorldResourcesList, SimulationInterfaces::FailedResult> availableWorlds;
        const SimulationInterfaces::GetWorldsRequest getWorldsRequest = ConvertROS2Request(request);

        SimulationInterfaces::LevelManagerRequestBus::BroadcastResult(
            availableWorlds, &SimulationInterfaces::LevelManagerRequests::GetAvailableWorlds, getWorldsRequest);

        if (!availableWorlds.IsSuccess())
        {
            response.result.result = availableWorlds.GetError().m_errorCode;
            response.result.error_message = availableWorlds.GetError().m_errorString.c_str();
        }
        else
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            const auto& worldsList = availableWorlds.GetValue();
            AZStd::ranges::transform(
                worldsList, AZStd::back_inserter(response.worlds), &SimulationInterfaces::Utils::ConvertToRos2WorldResource);
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
