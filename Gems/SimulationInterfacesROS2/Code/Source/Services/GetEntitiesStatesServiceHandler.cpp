/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntitiesStatesServiceHandler.h"
#include "Utils/Utils.h"
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <std_msgs/msg/header.hpp>

namespace SimulationInterfacesROS2
{
    GetEntitiesStatesServiceHandler::GetEntitiesStatesServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_getEntitiesStatesService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](const Request::SharedPtr request, Response::SharedPtr response)
            {
                *response = HandleServiceRequest(*request);
            });
    }

    GetEntitiesStatesServiceHandler::~GetEntitiesStatesServiceHandler()
    {
        if (m_getEntitiesStatesService)
        {
            m_getEntitiesStatesService.reset();
        }
    }

    GetEntitiesStatesServiceHandler::Response GetEntitiesStatesServiceHandler::HandleServiceRequest(const Request& request)
    {
        AZ::Outcome<SimulationInterfaces::MultipleEntitiesStates, SimulationInterfaces::FailedResult> outcome;

        GetEntitiesStatesServiceHandler::Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;

        const auto getFilterResult = Utils::GetEntityFiltersFromRequest<Request>(request);
        if (!getFilterResult.IsSuccess())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = getFilterResult.GetError().c_str();
            return response;
        }
        SimulationInterfaces::EntityFilters filter = getFilterResult.GetValue();
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntitiesStates, filter);

        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
            response.result.error_message = failedResult.error_string.c_str();
            return response;
        }

        const auto& multipleEntitiesStates = outcome.GetValue();
        std::vector<std::string> stdEntities;
        std::vector<simulation_interfaces::msg::EntityState> stdEntityStates;

        AZStd::transform(
            multipleEntitiesStates.begin(),
            multipleEntitiesStates.end(),
            std::back_inserter(stdEntities),
            [](const auto& pair)
            {
                return pair.first.c_str();
            });
        AZStd::transform(
            multipleEntitiesStates.begin(),
            multipleEntitiesStates.end(),
            std::back_inserter(stdEntityStates),
            [](const auto& pair)
            {
                const SimulationInterfaces::EntityState& entityState = pair.second;
                simulation_interfaces::msg::EntityState simulationInterfaceEntityState;
                std_msgs::msg::Header header;
                header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
                header.frame_id = ROS2::ROS2Interface::Get()->GetNode()->get_name();
                simulationInterfaceEntityState.header = header;
                simulationInterfaceEntityState.pose = ROS2::ROS2Conversions::ToROS2Pose(entityState.m_pose);
                simulationInterfaceEntityState.twist.linear = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twist_linear);
                simulationInterfaceEntityState.twist.angular = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twist_angular);
                return simulationInterfaceEntityState;
            });
        response.entities = stdEntities;
        response.states = stdEntityStates;

        return response;
    }
} // namespace SimulationInterfacesROS2
