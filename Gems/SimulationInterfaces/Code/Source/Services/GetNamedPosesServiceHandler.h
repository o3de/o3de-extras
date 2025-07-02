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
#include <simulation_interfaces/srv/get_named_poses.hpp>

namespace ROS2SimulationInterfaces
{

    class GetNamedPosesServiceHandler : public ROS2ServiceBase<simulation_interfaces::srv::GetNamedPoses>
    {
    public:
        AZStd::string_view GetTypeName() const override
        {
            return "GetNamedPoses";
        }

        AZStd::string_view GetDefaultName() const override
        {
            return "get_named_poses";
        }
        AZStd::unordered_set<SimulationFeatureType> GetProvidedFeatures() override;

        AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) override;

    private:
    };

} // namespace ROS2SimulationInterfaces
