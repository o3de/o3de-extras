/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SetEntityStateServiceHandler.h"
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{
    SetEntityStateServiceHandler::SetEntityStateServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName)
    {
        const std::string serviceNameStr(std::string_view(serviceName.data(), serviceName.size()));
        m_setEntityStateService = node->create_service<ServiceType>(
            serviceNameStr,
            [this](const Request::SharedPtr request, Response::SharedPtr response)
            {
                *response = HandleServiceRequest(*request);
            });
    }

    SetEntityStateServiceHandler::~SetEntityStateServiceHandler()
    {
        if (m_setEntityStateService)
        {
            m_setEntityStateService.reset();
        }
    }

    AZStd::unordered_set<AZ::u8> SetEntityStateServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::ENTITY_STATE_SETTING };
    }

    SetEntityStateServiceHandler::Response SetEntityStateServiceHandler::HandleServiceRequest(const Request& request)
    {
        AZ::Outcome<void, SimulationInterfaces::FailedResult> outcome;
        AZStd::string entityName = request.entity.c_str();
        SimulationInterfaces::EntityState entityState;
        entityState.m_pose = ROS2::ROS2Conversions::FromROS2Pose(request.state.pose);
        entityState.m_twist_angular = ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.angular);
        entityState.m_twist_linear = ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.linear);

        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::SetEntityState, entityName, entityState);

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
            response.result.error_message = failedResult.error_string.c_str();
            return response;
        }

        return response;
    }
} // namespace SimulationInterfacesROS2
