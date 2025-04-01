/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetSpawnablesServiceHandler.h"
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{
    GetSpawnablesServiceHandler::GetSpawnablesServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_getSpawnablesService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](const Request::SharedPtr request, Response::SharedPtr response)
            {
                *response = HandleServiceRequest(*request);
            });
    }

    GetSpawnablesServiceHandler::~GetSpawnablesServiceHandler()
    {
        if (m_getSpawnablesService)
        {
            m_getSpawnablesService.reset();
        }
    }

    GetSpawnablesServiceHandler::Response GetSpawnablesServiceHandler::HandleServiceRequest(const Request& request)
    {
        AZStd::vector<SimulationInterfaces::Spawnable> spawnables;
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            spawnables, &SimulationInterfaces::SimulationEntityManagerRequests::GetSpawnables);
        GetSpawnablesServiceHandler::Response response;
        std::vector<simulation_interfaces::msg::Spawnable> simSpawnables;
        AZStd::transform(
            spawnables.begin(),
            spawnables.end(),
            AZStd::back_inserter(simSpawnables),
            [](const SimulationInterfaces::Spawnable& spawnable)
            {
                simulation_interfaces::msg::Spawnable simSpawnable;
                simSpawnable.uri = spawnable.m_uri.c_str();
                simSpawnable.description = spawnable.m_description.c_str();
                return simSpawnable;
            });
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        response.spawnables = simSpawnables;

        return response;
    }
} // namespace SimulationInterfacesROS2