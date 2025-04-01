/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "DeleteEntityServiceHandler.h"

#include <AzCore/std/string/string.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{
    DeleteEntityServiceHandler::DeleteEntityServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_deleteEntityService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](const Request::SharedPtr request, Response::SharedPtr response)
            {
                *response = HandleServiceRequest(*request);
            });
    }

    DeleteEntityServiceHandler::~DeleteEntityServiceHandler()
    {
        if (m_deleteEntityService)
        {
            m_deleteEntityService.reset();
        }
    }

    DeleteEntityServiceHandler::Response DeleteEntityServiceHandler::HandleServiceRequest(const Request& request)
    {
        AZStd::string entityName = request.entity.c_str();
        bool result = false;
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            result, &SimulationInterfaces::SimulationEntityManagerRequests::DeleteEntity, entityName);
        DeleteEntityServiceHandler::Response response;

        response.result.result =
            result ? simulation_interfaces::msg::Result::RESULT_OK : simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;

        return response;
    }
} // namespace SimulationInterfacesROS2