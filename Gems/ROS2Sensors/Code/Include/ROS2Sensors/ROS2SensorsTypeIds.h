/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace ROS2Sensors
{
    // System Component TypeIds
    inline constexpr const char* ROS2SensorsSystemComponentTypeId = "{ECA6DB16-1F77-4098-956E-F1463B95AE15}";
    inline constexpr const char* ROS2SensorsEditorSystemComponentTypeId = "{F6A1AE53-0ACE-4425-AABB-C338A74CA6F1}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2SensorsModuleInterfaceTypeId = "{2A5B29DE-6056-4553-8C50-F7DDC8F1D4D0}";
    inline constexpr const char* ROS2SensorsModuleTypeId = "{5607AC61-7364-4690-9DF6-6265EAC0C1DB}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2SensorsEditorModuleTypeId = ROS2SensorsModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2SensorsRequestsTypeId = "{ACFDC553-B7E3-475C-91A3-26AB01C012AF}";
} // namespace ROS2Sensors
