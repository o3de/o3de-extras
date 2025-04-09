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
#include <simulation_interfaces/srv/spawn_entity.hpp>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/Component/EntityId.h>

namespace SimulationInterfacesROS2
{
    class SpawnEntityServiceHandler : public ROS2HandlerBase<simulation_interfaces::srv::SpawnEntity>
    {
    public:
        AZStd::string_view GetTypeName() const override
        {
            return "SpawnEntity";
        }

        AZStd::string_view GetDefaultName() const override
        {
            return "spawn_entity";
        }
        AZStd::unordered_set<AZ::u8> GetProvidedFeatures() override;

        AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) override;

    private:
        void SetRos2Namespace(const AZStd::string& namespacePrefix, const AZStd::vector<AZ::EntityId>& entityIds);

    };

} // namespace SimulationInterfacesROS2
