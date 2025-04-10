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
#include <simulation_interfaces/srv/step_simulation.hpp>
#include <AzCore/Component/TickBus.h>
namespace SimulationInterfacesROS2
{

    class StepSimulationServiceHandler
        : public ROS2ServiceBase<simulation_interfaces::srv::StepSimulation>
        , private AZ::TickBus::Handler
    {
    public:
        AZStd::string_view GetTypeName() const override
        {
            return "StepSimulation";
        }

        AZStd::string_view GetDefaultName() const override
        {
            return "step_simulation";
        }
        AZStd::unordered_set<AZ::u8> GetProvidedFeatures() override;

        AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) override;

    private:
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
    };

} // namespace SimulationInterfacesROS2
