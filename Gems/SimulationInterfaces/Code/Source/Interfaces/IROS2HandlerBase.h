/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string_view.h>
#include <SimulationInterfaces/ROS2SimulationInterfacesRequestBus.h>
#include <rclcpp/rclcpp.hpp>

namespace ROS2SimulationInterfaces
{
    // common interface to store all simulation feature ros2  handlers in common container
    class IROS2HandlerBase
    {
    public:
        using SimulationFeatureType = ROS2SimulationInterfaces::SimulationFeatureType;
        virtual ~IROS2HandlerBase() = default;
        virtual AZStd::unordered_set<SimulationFeatureType> GetProvidedFeatures() = 0;
        virtual AZStd::string_view GetTypeName() const = 0;
        virtual AZStd::string_view GetDefaultName() const = 0;
        virtual void Initialize(rclcpp::Node::SharedPtr& node) = 0;
    };
} // namespace ROS2SimulationInterfaces
