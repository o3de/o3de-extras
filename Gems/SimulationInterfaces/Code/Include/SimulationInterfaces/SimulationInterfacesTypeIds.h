/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace SimulationInterfaces
{
    // System Component TypeIds
    inline constexpr const char* SimulationEntitiesManagerTypeId = "{4BF53AF2-A295-4F99-A166-F85FBFBDC077}";
    inline constexpr const char* SimulationEntitiesManagerEditorTypeId = "{B035007B-BAD3-40FA-880F-F45054A4C232}";

    inline constexpr const char* SimulationManagerTypeId = "{5BB34EB0-1263-4DA1-A35C-CE613A088F4B}";
    inline constexpr const char* SimulationManagerEditorTypeId = "{2CC8D67B-CFD3-4E89-AAF0-8935640B51C1}";

    // Module derived classes TypeIds
    inline constexpr const char* SimulationInterfacesModuleInterfaceTypeId = "{675797BF-E5D5-438A-BF86-4B4554F09CEF}";
    inline constexpr const char* SimulationInterfacesModuleTypeId = "{8D6741FD-3105-4CB0-9700-152123B6D135}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* SimulationInterfacesEditorModuleTypeId = SimulationInterfacesModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* SimulationInterfacesRequestsTypeId = "{6818E5E3-BBF5-41BD-96BB-7AF57CCC7528}";

} // namespace SimulationInterfaces
