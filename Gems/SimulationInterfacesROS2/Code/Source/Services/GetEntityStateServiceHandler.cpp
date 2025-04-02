/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntityStateServiceHandler.h"
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <std_msgs/msg/header.hpp>

namespace SimulationInterfacesROS2
{

    GetEntityStateServiceHandler::GetEntityStateServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_getEntityStateService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](const Request::SharedPtr request, Response::SharedPtr response)
            {
                *response = HandleServiceRequest(*request);
            });
    }

    GetEntityStateServiceHandler::~GetEntityStateServiceHandler()
    {
        if (m_getEntityStateService)
        {
            m_getEntityStateService.reset();
        }
    }

    GetEntityStateServiceHandler::Response GetEntityStateServiceHandler::HandleServiceRequest(const Request& request)
    {
        AZStd::string entityName = request.entity.c_str();
        AZ::Outcome<SimulationInterfaces::EntityState, SimulationInterfaces::FailedResult> outcome;

        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntityState, entityName);

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
            response.result.error_message = failedResult.error_string.c_str();
            return response;
        }

        const auto& entityState = outcome.GetValue();
        simulation_interfaces::msg::EntityState entityStateMsg;
        std_msgs::msg::Header header;
        header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        header.frame_id = ROS2::ROS2Interface::Get()->GetNode()->get_name();
        entityStateMsg.header = header;
        entityStateMsg.pose = ROS2::ROS2Conversions::ToROS2Pose(entityState.m_pose);
        entityStateMsg.twist.linear = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twist_linear);
        entityStateMsg.twist.angular = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twist_angular);

        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        response.state = entityStateMsg;
        return response;
    }
} // namespace SimulationInterfacesROS2
