/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntitiesStatesServiceHandler.h"
#include "AzCore/std/smart_ptr/make_shared.h"
#include "ROS2/Utilities/ROS2Conversions.h"
#include "Utils/Utils.h"
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/ROS2Bus.h>
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
        AZStd::unordered_map<AZStd::string, SimulationInterfaces::EntityState> entitiesStates;

        GetEntitiesStatesServiceHandler::Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;

        const auto getFilterResult = Utils::GetEntityFilterFromRequest<Request>(request);
        if (!getFilterResult.IsSuccess())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = getFilterResult.GetError().c_str();
            return response;
        }
        SimulationInterfaces::EntityFilters filter = getFilterResult.GetValue();
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            entitiesStates, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntitiesStates, filter);
        std::vector<std::string> stdEntities;
        std::vector<simulation_interfaces::msg::EntityState> stdEntityStates;

        AZStd::transform(
            entitiesStates.begin(),
            entitiesStates.end(),
            std::back_inserter(stdEntities),
            [](const auto& pair)
            {
                return pair.first.c_str();
            });
        AZStd::transform(
            entitiesStates.begin(),
            entitiesStates.end(),
            std::back_inserter(stdEntityStates),
            [](const auto& pair)
            {
                const SimulationInterfaces::EntityState entityState = pair.second;
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