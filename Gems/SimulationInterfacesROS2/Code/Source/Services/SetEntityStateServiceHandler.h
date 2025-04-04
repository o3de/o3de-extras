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
#include <simulation_interfaces/srv/set_entity_state.hpp>

namespace SimulationInterfacesROS2
{

    class SetEntityStateServiceHandler : public ROS2HandlerBase
    {
    public:
        using ServiceType = simulation_interfaces::srv::SetEntityState;
        using Request = ServiceType::Request;
        using Response = ServiceType::Response;

        SetEntityStateServiceHandler() = delete;
        SetEntityStateServiceHandler(rclcpp::Node::SharedPtr& node, AZStd::string_view serviceName);
        ~SetEntityStateServiceHandler();
        AZStd::unordered_set<AZ::u8> GetProvidedFeatures() override;
        Response HandleServiceRequest(const Request& request);

    private:
        rclcpp::Service<ServiceType>::SharedPtr m_setEntityStateService;
    };

} // namespace SimulationInterfacesROS2
