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
            [this](
                const ServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Request> request)
            {
                HandleServiceRequest(service_handle, header, request);
            });
    }

    DeleteEntityServiceHandler::~DeleteEntityServiceHandler()
    {
        if (m_deleteEntityService)
        {
            m_deleteEntityService.reset();
        }
    }

    void DeleteEntityServiceHandler::HandleServiceRequest(
        const ServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Request> request)
    {
        AZStd::string entityName = request->entity.c_str();
        SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationEntityManagerRequests::DeleteEntity,
            entityName,
            [service_handle, header](const AZ::Outcome<void, SimulationInterfaces::FailedResult>& outcome)
            {
                Response response;
                if (outcome.IsSuccess())
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                }
                else
                {
                    const auto& failedResult = outcome.GetError();
                    response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
                    response.result.error_message = failedResult.error_string.c_str();
                }
                service_handle->send_response(*header, response);
            });
    }
} // namespace SimulationInterfacesROS2
