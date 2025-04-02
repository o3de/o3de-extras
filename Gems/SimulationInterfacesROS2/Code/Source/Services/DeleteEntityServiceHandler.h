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
        using ServiceHandle = std::shared_ptr<rclcpp::Service<ServiceType>>;

        DeleteEntityServiceHandler() = delete;
        DeleteEntityServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName);
        ~DeleteEntityServiceHandler();

        void HandleServiceRequest(
            const ServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<Request> request);

    private:
        rclcpp::Service<ServiceType>::SharedPtr m_deleteEntityService;
    };

} // namespace SimulationInterfacesROS2
