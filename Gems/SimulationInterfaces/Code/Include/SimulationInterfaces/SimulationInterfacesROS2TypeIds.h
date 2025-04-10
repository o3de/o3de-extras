/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace SimulationInterfacesROS2
{
    // System Component TypeIds
    inline constexpr const char* SimulationInterfacesROS2SystemComponentTypeId = "{9CD6E9FA-5C17-454C-B8FA-033DF572B160}";
    inline constexpr const char* SimulationInterfacesROS2EditorSystemComponentTypeId = "{AF5BE964-4B5F-49A4-A308-0B6077E5BB26}";

    // Module derived classes TypeIds
    inline constexpr const char* SimulationInterfacesROS2ModuleInterfaceTypeId = "{2F1ED7E1-6808-420D-939F-7D5C9CBFB3C9}";
    inline constexpr const char* SimulationInterfacesROS2ModuleTypeId = "{4002B625-F939-44AC-845B-820B20AFC6C5}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* SimulationInterfacesROS2EditorModuleTypeId = SimulationInterfacesROS2ModuleTypeId;

    // API TypeIds
    inline constexpr const char* SimulationInterfacesROS2RequestBusTypeId = "{00d08870-e329-4bd7-bb8c-f67fe369de92}";
} // namespace SimulationInterfacesROS2
