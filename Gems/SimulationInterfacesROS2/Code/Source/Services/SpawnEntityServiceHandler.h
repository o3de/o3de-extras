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
#include <simulation_interfaces/srv/spawn_entity.hpp>

namespace SimulationInterfacesROS2
{

    class SpawnEntityServiceHandler
    {
    public:
        using ServiceType = simulation_interfaces::srv::SpawnEntity;
        using SpawnEntityRequest = ServiceType::Request;
        using SpawnEntityResponse = ServiceType::Response;
        using SpawnEntityServiceHandle = std::shared_ptr<rclcpp::Service<ServiceType>>;

        SpawnEntityServiceHandler() = delete;
        SpawnEntityServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName);
        ~SpawnEntityServiceHandler();

    private:
        void HandleSpawnRequest(
            const SpawnEntityServiceHandle service_handle,
            const std::shared_ptr<rmw_request_id_t> header,
            const std::shared_ptr<SpawnEntityRequest> request);

        rclcpp::Service<ServiceType>::SharedPtr m_spawnEntityService;
    };

} // namespace SimulationInterfacesROS2
