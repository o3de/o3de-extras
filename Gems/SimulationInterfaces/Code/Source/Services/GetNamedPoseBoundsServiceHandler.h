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
#include <simulation_interfaces/srv/get_named_pose_bounds.hpp>

namespace ROS2SimulationInterfaces
{

    class GetNamedPoseBoundsServiceHandler
        : public ROS2::ROS2ServiceBase<simulation_interfaces::srv::GetNamedPoseBounds>
        , public ISimulationFeaturesBase
    {
    public:
        AZStd::string_view GetTypeName() const override
        {
            return "GetNamedPoseBounds";
        }

        AZStd::string_view GetDefaultName() const override
        {
            return "get_named_pose_bounds";
        }
        AZStd::unordered_set<SimulationFeatureType> GetProvidedFeatures() override;

        AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) override;

    private:
    };

} // namespace ROS2SimulationInterfaces
