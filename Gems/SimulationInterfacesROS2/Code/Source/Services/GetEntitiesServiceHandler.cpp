/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntitiesServiceHandler.h"
#include "Utils/Utils.h"
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{
    GetEntitiesServiceHandler::GetEntitiesServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_getEntitiesService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](const Request::SharedPtr request, Response::SharedPtr response)
            {
                *response = HandleServiceRequest(*request);
            });
    }

    GetEntitiesServiceHandler::~GetEntitiesServiceHandler()
    {
        if (m_getEntitiesService)
        {
            m_getEntitiesService.reset();
        }
    }

    GetEntitiesServiceHandler::Response GetEntitiesServiceHandler::HandleServiceRequest(const Request& request)
    {
        AZ::Outcome<SimulationInterfaces::EntityNameList, SimulationInterfaces::FailedResult> outcome;

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;

        const auto getFilterResult = Utils::GetEntityFiltersFromRequest<Request>(request);
        if (!getFilterResult.IsSuccess())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = getFilterResult.GetError().c_str();
            return response;
        }
        const SimulationInterfaces::EntityFilters filter = getFilterResult.GetValue();
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntities, filter);
        std::vector<std::string>& stdEntities = response.entities;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
            response.result.error_message = failedResult.error_string.c_str();
            return response;
        }

        const auto& entityNameList = outcome.GetValue();
        AZStd::transform(
            entityNameList.begin(),
            entityNameList.end(),
            std::back_inserter(stdEntities),
            [](const AZStd::string& entityName)
            {
                return entityName.c_str();
            });
        response.entities = stdEntities;

        return response;
    }
} // namespace SimulationInterfacesROS2
