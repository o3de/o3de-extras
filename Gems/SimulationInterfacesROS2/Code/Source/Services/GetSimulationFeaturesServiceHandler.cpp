/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetSimulationFeaturesServiceHandler.h"
#include "SimulationInterfacesROS2/SimulationInterfacesROS2RequestBus.h"
#include <AzCore/base.h>
#include <AzCore/std/containers/unordered_set.h>

namespace SimulationInterfacesROS2
{

    GetSimulationFeaturesServiceHandler::GetSimulationFeaturesServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_getSimulationFeaturesService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](const Request::SharedPtr request, Response::SharedPtr response)
            {
                *response = HandleServiceRequest(*request);
            });
    }

    GetSimulationFeaturesServiceHandler::~GetSimulationFeaturesServiceHandler()
    {
        if (m_getSimulationFeaturesService)
        {
            m_getSimulationFeaturesService.reset();
        }
    }

    AZStd::unordered_set<AZ::u8> GetSimulationFeaturesServiceHandler::GetProvidedFeatures()
    {
        // standard doesn't define specific feature id for this service
        return AZStd::unordered_set<AZ::u8>{};
    }

    GetSimulationFeaturesServiceHandler::Response GetSimulationFeaturesServiceHandler::HandleServiceRequest(const Request& request)
    {
        // call bus to get simulation features in SimulationInterfacesROS2 Gem side
        AZStd::unordered_set<AZ::u8> ros2Interfaces;
        SimulationInterfacesROS2RequestBus::BroadcastResult(ros2Interfaces, &SimulationInterfacesROS2Requests::GetSimulationFeatures);
        // call bus to get simulation features on SimulationInterfaces Gem  side

        // create common features and return it;

        Response response;
        response.features.features.insert(response.features.features.end(), ros2Interfaces.begin(), ros2Interfaces.end());
        return response;
    }
} // namespace SimulationInterfacesROS2
