/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SetEntityStateServiceHandler.h"

#include "ROS2/Utilities/ROS2Conversions.h"

#include <AzCore/std/smart_ptr/shared_ptr.h>
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

    SetEntityStateServiceHandler::Response SetEntityStateServiceHandler::HandleServiceRequest(const Request& request)
    {
        bool result = false;
        AZStd::string entityName = request.entity.c_str();
        SimulationInterfaces::EntityState entityState;
        entityState.m_pose = ROS2::ROS2Conversions::FromROS2Pose(request.state.pose);
        entityState.m_twist_angular = ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.angular);
        entityState.m_twist_linear = ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.linear);

        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            result, &SimulationInterfaces::SimulationEntityManagerRequests::SetEntityState, entityName, entityState);

        SetEntityStateServiceHandler::Response response;
        response.result.result =
            result ? simulation_interfaces::msg::Result::RESULT_OK : simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
        return response;
    }
} // namespace SimulationInterfacesROS2