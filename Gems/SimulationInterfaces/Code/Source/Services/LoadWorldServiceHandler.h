/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string_view.h>
#include <Interfaces/ISimulationFeaturesBase.h>
#include <ROS2/Handlers/ROS2ServiceBase.h>
#include <simulation_interfaces/srv/load_world.hpp>

namespace ROS2SimulationInterfaces
{

    class LoadWorldServiceHandler
        : public ROS2::ROS2ServiceBase<simulation_interfaces::srv::LoadWorld>
        , public ISimulationFeaturesBase
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
