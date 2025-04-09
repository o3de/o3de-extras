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
#include <SimulationInterfacesROS2/SimulationInterfacesROS2RequestBus.h>

namespace SimulationInterfacesROS2
{

    AZStd::unordered_set<AZ::u8> GetSimulationFeaturesServiceHandler::GetProvidedFeatures()
    {
        // standard doesn't define specific feature id for this service
        return AZStd::unordered_set<AZ::u8>{};
    }

    AZStd::optional<GetSimulationFeaturesServiceHandler::Response> GetSimulationFeaturesServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        // call bus to get simulation features in SimulationInterfacesROS2 Gem side
        AZStd::unordered_set<AZ::u8> ros2Interfaces;
        SimulationInterfacesROS2RequestBus::BroadcastResult(ros2Interfaces, &SimulationInterfacesROS2Requests::GetSimulationFeatures);
        // call bus to get simulation features on SimulationInterfaces Gem  side
        AZStd::unordered_set<SimulationInterfaces::SimulationFeatures> o3deInterfaces;
        SimulationInterfaces::SimulationFeaturesAggregatorRequestBus::BroadcastResult(
            o3deInterfaces, &SimulationInterfaces::SimulationFeaturesAggregatorRequests::GetSimulationFeatures);
        // create common features and return it;
        // common features are logical AND between two sets
        AZStd::unordered_set<AZ::u8> commonFeatures;
        commonFeatures.insert(ros2Interfaces.begin(), ros2Interfaces.end());
        for (auto id : o3deInterfaces)
        {
            commonFeatures.insert(static_cast<AZ::u8>(id));
        }
        Response response;
        for (auto id : commonFeatures)
        {
            if (ros2Interfaces.contains(id) && o3deInterfaces.contains(SimulationInterfaces::SimulationFeatures(id)))
            {
                response.features.features.emplace_back(id);
            }
        }

        return response;
    }
} // namespace SimulationInterfacesROS2
