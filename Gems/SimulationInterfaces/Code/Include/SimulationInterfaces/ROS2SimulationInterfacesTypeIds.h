/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace ROS2SimulationInterfaces
{
    // System Component TypeIds
    inline constexpr const char* ROS2SimulationInterfacesSystemComponentTypeId = "{9CD6E9FA-5C17-454C-B8FA-033DF572B160}";
    inline constexpr const char* ROS2SimulationInterfacesEditorSystemComponentTypeId = "{AF5BE964-4B5F-49A4-A308-0B6077E5BB26}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2SimulationInterfacesModuleInterfaceTypeId = "{2F1ED7E1-6808-420D-939F-7D5C9CBFB3C9}";
    inline constexpr const char* ROS2SimulationInterfacesModuleTypeId = "{4002B625-F939-44AC-845B-820B20AFC6C5}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2SimulationInterfacesEditorModuleTypeId = ROS2SimulationInterfacesModuleTypeId;

    // API TypeIds
    inline constexpr const char* ROS2SimulationInterfacesRequestBusTypeId = "{00D08870-E329-4BD7-BB8C-F67FE369DE92}";

    // Internal API TypeIds
    inline constexpr const char* TFInterfaceTypeId = "{BC0C8C08-DC47-44AB-9A84-D853751D15F1}";
} // namespace ROS2SimulationInterfaces
