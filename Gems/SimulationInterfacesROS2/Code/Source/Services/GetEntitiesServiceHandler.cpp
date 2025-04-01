/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntitiesServiceHandler.h"

#include "AzCore/std/smart_ptr/make_shared.h"
#include "ROS2/Utilities/ROS2Conversions.h"

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
        AZStd::vector<AZStd::string> entities;

        GetEntitiesServiceHandler::Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;

        const auto getFilterResult = Utils::GetEntityFilterFromRequest<Request>(request);
        if (!getFilterResult.IsSuccess())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = getFilterResult.GetError().c_str();
            return response;
        }
        const SimulationInterfaces::EntityFilters filter = getFilterResult.GetValue();
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            entities, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntities, filter);
        std::vector<std::string> stdEntities;
        AZStd::transform(
            entities.begin(),
            entities.end(),
            std::back_inserter(stdEntities),
            [](const AZStd::string& entityName)
            {
                return entityName.c_str();
            });
        response.entities = stdEntities;

        return response;
    }
} // namespace SimulationInterfacesROS2