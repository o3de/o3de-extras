/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SpawnEntityServiceHandler.h"

#include "ROS2/Utilities/ROS2Conversions.h"
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{
    SpawnEntityServiceHandler::SpawnEntityServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_spawnEntityService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](
                const SpawnEntityServiceHandle service_handle,
                const std::shared_ptr<rmw_request_id_t> header,
                const std::shared_ptr<SpawnEntityRequest> request)
            {
                this->HandleSpawnRequest(service_handle, header, request);
            });
    }

    SpawnEntityServiceHandler::~SpawnEntityServiceHandler()
    {
        if (m_spawnEntityService)
        {
            m_spawnEntityService.reset();
        }
    }

    void SpawnEntityServiceHandler::HandleSpawnRequest(
        const SpawnEntityServiceHandle service_handle,
        const std::shared_ptr<rmw_request_id_t> header,
        const std::shared_ptr<SpawnEntityRequest> request)
    {
        AZStd::string name = request->name.c_str();
        AZStd::string uri = request->uri.c_str();
        AZStd::string entityNamespace = request->entity_namespace.c_str();
        AZ::Transform initialPose = ROS2::ROS2Conversions::FromROS2Pose(request->initial_pose.pose);
        SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationEntityManagerRequests::SpawnEntity,
            name,
            uri,
            entityNamespace,
            initialPose,
            [service_handle, header](const AZ::Outcome<AZStd::string, AZStd::string>& outcome)
            {
                SpawnEntityResponse response;
                if (outcome.IsSuccess())
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                    response.entity_name = outcome.GetValue().c_str();
                }
                else
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
                    response.result.error_message = outcome.GetError().c_str();
                }
                service_handle->send_response(*header, response);
            });
    }

} // namespace SimulationInterfacesROS2