/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2ServiceBase.h"
#include <AzCore/std/string/string_view.h>
#include <simulation_interfaces/srv/load_world.hpp>

namespace ROS2SimulationInterfaces
{

    class LoadWorldServiceHandler : public ROS2ServiceBase<simulation_interfaces::srv::LoadWorld>
    {
    public:
        AZStd::string_view GetTypeName() const override
        {
            return "LoadWorld";
        }

        AZStd::string_view GetDefaultName() const override
        {
            return "load_world";
        }
        AZStd::unordered_set<SimulationFeatureType> GetProvidedFeatures() override;

        AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) override;
    };

} // namespace ROS2SimulationInterfaces
