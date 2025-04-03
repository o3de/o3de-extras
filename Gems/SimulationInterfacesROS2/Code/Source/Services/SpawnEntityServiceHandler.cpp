/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SpawnEntityServiceHandler.h"
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{
    SpawnEntityServiceHandler::SpawnEntityServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_spawnEntityService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](
                const ServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Request> request)
            {
                HandleServiceRequest(service_handle, header, request);
            });
    }

    SpawnEntityServiceHandler::~SpawnEntityServiceHandler()
    {
        if (m_spawnEntityService)
        {
            m_spawnEntityService.reset();
        }
    }

    void SpawnEntityServiceHandler::HandleServiceRequest(
        const ServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Request> request)
    {
        const AZStd::string_view name {request->name.c_str(), request->name.size()};
        const AZStd::string_view uri {request->uri.c_str(), request->uri.size()};
        const AZStd::string_view entityNamespace {request->entity_namespace.c_str(), request->entity_namespace.size()};
        const AZ::Transform initialPose = ROS2::ROS2Conversions::FromROS2Pose(request->initial_pose.pose);
        SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationEntityManagerRequests::SpawnEntity,
            name,
            uri,
            entityNamespace,
            initialPose,
            [service_handle, header](const AZ::Outcome<AZStd::string, SimulationInterfaces::FailedResult>& outcome)
            {
                Response response;
                if (outcome.IsSuccess())
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                    response.entity_name = outcome.GetValue().c_str();
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
