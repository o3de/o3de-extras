/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LoadWorldServiceHandler.h"
#include "SimulationInterfaces/WorldResource.h"
#include "Utils/Utils.h"
#include <AzCore/std/string/string.h>
#include <SimulationInterfaces/LevelManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{
    namespace
    {
        [[nodiscard]] SimulationInterfaces::LoadWorldRequest ConvertROS2Request(const LoadWorldServiceHandler::Request& ros2Request)
        {
            SimulationInterfaces::LoadWorldRequest request;
            request.failOnUnsupportedElement = ros2Request.fail_on_unsupported_element;
            request.ignoreMissingOrUnsupportedAssets = ros2Request.ignore_missing_or_unsupported_assets;
            request.levelResource.m_uri = ros2Request.world_resource.uri.c_str();
            request.levelResource.m_resourceString = ros2Request.world_resource.resource_string.c_str();
            return request;
        }
    } // namespace

    AZStd::unordered_set<SimulationFeatureType> LoadWorldServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::WORLD_LOADING };
    }

    AZStd::optional<LoadWorldServiceHandler::Response> LoadWorldServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZ::Outcome<SimulationInterfaces::WorldResource, SimulationInterfaces::FailedResult> loadedWorld;
        SimulationInterfaces::LoadWorldRequest loadWorldRequest = ConvertROS2Request(request);
        SimulationInterfaces::LevelManagerRequestBus::BroadcastResult(
            loadedWorld, &SimulationInterfaces::LevelManagerRequests::LoadWorld, loadWorldRequest);
        Response response;
        if (!loadedWorld.IsSuccess())
        {
            response.result.result = loadedWorld.GetError().m_errorCode;
            response.result.error_message = loadedWorld.GetError().m_errorString.c_str();
        }
        else
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            response.world = Utils::ConvertWorldResource(loadedWorld.GetValue());
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
