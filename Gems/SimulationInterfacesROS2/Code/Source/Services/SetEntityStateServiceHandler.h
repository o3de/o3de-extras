/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "Services/ROS2ServiceBaseClass.h"
#include <AzCore/std/string/string_view.h>
#include <simulation_interfaces/srv/set_entity_state.hpp>

namespace SimulationInterfacesROS2
{

    class SetEntityStateServiceHandler : public ROS2ServiceBase<simulation_interfaces::srv::SetEntityState>
    {
    public:
        AZStd::string_view GetTypeName() const override
        {
            return "SetEntityState";
        }

        AZStd::string_view GetDefaultName() const override
        {
            return "set_entity_state";
        }
        AZStd::unordered_set<AZ::u8> GetProvidedFeatures() override;

        AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) override;

    private:
    };

} // namespace SimulationInterfacesROS2
