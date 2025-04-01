/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string_view.h>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interfaces/srv/get_entity_state.hpp>

namespace SimulationInterfacesROS2
{

    class GetEntityStateServiceHandler
    {
    public:
        using ServiceType = simulation_interfaces::srv::GetEntityState;
        using Request = ServiceType::Request;
        using Response = ServiceType::Response;

        GetEntityStateServiceHandler() = delete;
        GetEntityStateServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName);
        ~GetEntityStateServiceHandler();

        Response HandleServiceRequest(const Request& request);

    private:
        rclcpp::Service<ServiceType>::SharedPtr m_getEntityStateService;
    };

} // namespace SimulationInterfacesROS2
