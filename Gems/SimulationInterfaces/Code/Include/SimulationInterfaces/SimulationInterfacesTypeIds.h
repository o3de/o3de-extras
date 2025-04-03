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

    inline constexpr const char* SimulationFeaturesAggregatorTypeId = "{5c5ae765-1776-4ba0-8e32-b66c8f4edafe}";
    inline constexpr const char* SimulationFeaturesAggregatorEditorTypeId = "{504a86ef-df0a-45ec-b69d-315ff4ec8121}";

    // Module derived classes TypeIds
    inline constexpr const char* SimulationInterfacesModuleInterfaceTypeId = "{675797BF-E5D5-438A-BF86-4B4554F09CEF}";
    inline constexpr const char* SimulationInterfacesModuleTypeId = "{8D6741FD-3105-4CB0-9700-152123B6D135}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* SimulationInterfacesEditorModuleTypeId = SimulationInterfacesModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* SimulationInterfacesRequestsTypeId = "{6818E5E3-BBF5-41BD-96BB-7AF57CCC7528}";
    inline constexpr const char* SimulationManagerRequestsTypeId = "{056477BA-8153-4901-9401-0146A5E3E9ED}";
    inline constexpr const char* SimulationFeaturesAggregatorRequestsTypeId = "{099fd08b-b0e2-4705-9c35-cc09c8e45076}";
    inline constexpr const char* SimulationManagerNotificationsTypeId = "{0201067b-9d52-4ab7-9a45-284287f53b00}";

} // namespace SimulationInterfaces
