/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetSimulationFeaturesServiceHandler.h"
#include <AzCore/base.h>
#include <AzCore/std/containers/unordered_set.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> GetSimulationFeaturesServiceHandler::GetProvidedFeatures()
    {
        // standard doesn't define specific feature id for this service
        return AZStd::unordered_set<SimulationFeatureType>{};
    }

    AZStd::optional<GetSimulationFeaturesServiceHandler::Response> GetSimulationFeaturesServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        // call bus to get simulation features in ROS2SimulationInterfaces Gem side
        AZStd::unordered_set<SimulationFeatureType> ros2Interfaces;
        ROS2SimulationInterfacesRequestBus::BroadcastResult(ros2Interfaces, &ROS2SimulationInterfacesRequests::GetSimulationFeatures);
        // call bus to get simulation features on SimulationInterfaces Gem  side
        AZStd::unordered_set<SimulationInterfaces::SimulationFeatureType> o3deInterfaces;
        SimulationInterfaces::SimulationFeaturesAggregatorRequestBus::BroadcastResult(
            o3deInterfaces, &SimulationInterfaces::SimulationFeaturesAggregatorRequests::GetSimulationFeatures);
        // create common features and return it;
        // common features are logical AND between two sets
        AZStd::unordered_set<SimulationFeatureType> commonFeatures;
        commonFeatures.insert(ros2Interfaces.begin(), ros2Interfaces.end());
        commonFeatures.insert(o3deInterfaces.begin(), o3deInterfaces.end());

        Response response;
        for (auto id : commonFeatures)
        {
            if (ros2Interfaces.contains(id) && o3deInterfaces.contains(SimulationInterfaces::SimulationFeatureType(id)))
            {
                response.features.features.emplace_back(id);
            }
        }

        return response;
    }
} // namespace ROS2SimulationInterfaces
