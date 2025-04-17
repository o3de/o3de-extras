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
    inline constexpr const char* SimulationFeaturesAggregatorEditorTypeId = "{504A86EF-DF0A-45EC-B69D-315FF4EC8121}";

    inline constexpr const char* NamedPoseManagerTypeId = "{0F100078-C297-482C-BA56-644AB09F053F}";
    inline constexpr const char* NamedPoseManagerEditorTypeId = "{9E91A513-E1A5-43F0-B7B1-A759EEC23174}";

    // Module derived classes TypeIds
    inline constexpr const char* SimulationInterfacesModuleInterfaceTypeId = "{675797BF-E5D5-438A-BF86-4B4554F09CEF}";
    inline constexpr const char* SimulationInterfacesModuleTypeId = "{8D6741FD-3105-4CB0-9700-152123B6D135}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* SimulationInterfacesEditorModuleTypeId = SimulationInterfacesModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* SimulationInterfacesRequestsTypeId = "{6818E5E3-BBF5-41BD-96BB-7AF57CCC7528}";
    inline constexpr const char* SimulationManagerRequestsTypeId = "{056477BA-8153-4901-9401-0146A5E3E9ED}";
    inline constexpr const char* SimulationFeaturesAggregatorRequestsTypeId = "{099FD08B-B0E2-4705-9C35-CC09C8E45076}";
    inline constexpr const char* SimulationManagerNotificationsTypeId = "{0201067B-9D52-4AB7-9A45-284287F53B00}";
    inline constexpr const char* NamedPoseManagerRequestsTypeId = "{705280D6-7AB4-4586-A0A4-25AD71268279}";

} // namespace SimulationInterfaces
