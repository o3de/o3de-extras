/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

namespace WarehouseAutomation
{
    // System Component TypeIds
    inline constexpr const char* WarehouseAutomationSystemComponentTypeId = "{2A884131-6537-4B9E-8937-22AF2471C65B}";
    inline constexpr const char* WarehouseAutomationEditorSystemComponentTypeId = "{0630784F-5962-4104-9B0E-B23BED686C6C}";

    // Module derived classes TypeIds
    inline constexpr const char* WarehouseAutomationModuleInterfaceTypeId = "{6584FD8A-0FDA-48CD-A2E6-43F08CC9407E}";
    inline constexpr const char* WarehouseAutomationModuleTypeId = "{906B71F8-374D-4F8E-B8F2-EAFEFF863F7F}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* WarehouseAutomationEditorModuleTypeId = WarehouseAutomationModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* WarehouseAutomationRequestsTypeId = "{0A8FF6A3-0DE7-4DC0-BAFC-A9B61191A1F1}";
} // namespace WarehouseAutomation
