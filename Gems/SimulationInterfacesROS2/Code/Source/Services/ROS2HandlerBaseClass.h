/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/containers/unordered_set.h>
#include <simulation_interfaces/msg/simulator_features.hpp>
namespace SimulationInterfacesROS2
{
    //! base for each ros2 handler, forces declaration of features provided by the handler
    //! combined informations along all ROS 2 handlers gives information about simulation features
    //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
    using SimulationFeatures = simulation_interfaces::msg::SimulatorFeatures;
    class ROS2HandlerBase
    {
    public:
        virtual ~ROS2HandlerBase() = default;

        //! return features id defined by the handler, ids must follow the definition inside standard:
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
        virtual AZStd::unordered_set<AZ::u8> GetProvidedFeatures()
        {
            return {};
        };
    };
} // namespace SimulationInterfacesROS2
