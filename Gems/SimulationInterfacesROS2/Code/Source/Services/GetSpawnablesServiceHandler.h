/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "Services/ROS2HandlerBaseClass.h"
#include <AzCore/std/string/string_view.h>
#include <rclcpp/rclcpp.hpp>
#include <simulation_interfaces/srv/get_spawnables.hpp>

namespace SimulationInterfacesROS2
{

    class GetSpawnablesServiceHandler : public ROS2HandlerBase
    {
    public:
        using ServiceType = simulation_interfaces::srv::GetSpawnables;
        using Request = ServiceType::Request;
        using Response = ServiceType::Response;

        GetSpawnablesServiceHandler() = delete;
        GetSpawnablesServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName);
        ~GetSpawnablesServiceHandler();
        AZStd::unordered_set<AZ::u8> GetProvidedFeatures() override;
        Response HandleServiceRequest(const Request& request);

    private:
        rclcpp::Service<ServiceType>::SharedPtr m_getSpawnablesService;
    };

} // namespace SimulationInterfacesROS2
