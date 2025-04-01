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
#include <simulation_interfaces/srv/delete_entity.hpp>

namespace SimulationInterfacesROS2
{

    class DeleteEntityServiceHandler
    {
    public:
        using ServiceType = simulation_interfaces::srv::DeleteEntity;
        using Request = ServiceType::Request;
        using Response = ServiceType::Response;

        DeleteEntityServiceHandler() = delete;
        DeleteEntityServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName);
        ~DeleteEntityServiceHandler();

        Response HandleServiceRequest(const Request& request);

    private:
        rclcpp::Service<ServiceType>::SharedPtr m_deleteEntityService;
    };

} // namespace SimulationInterfacesROS2
