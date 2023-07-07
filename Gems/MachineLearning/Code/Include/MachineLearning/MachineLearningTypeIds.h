/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace MachineLearning
{
    // System Component TypeIds
    inline constexpr const char* MachineLearningSystemComponentTypeId = "{D1A87047-6088-42F6-9275-D6BCE2266322}";
    inline constexpr const char* MachineLearningEditorSystemComponentTypeId = "{EE7AB2E3-9A4B-45B8-93FE-B5FC6ED0D6FF}";

    // Module derived classes TypeIds
    inline constexpr const char* MachineLearningModuleInterfaceTypeId = "{DC0A66D9-3EE9-41D7-BDA0-40BE1AFE60C3}";
    inline constexpr const char* MachineLearningModuleTypeId = "{39CB5F37-F185-45F9-B0FA-20EB56797055}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* MachineLearningEditorModuleTypeId = MachineLearningModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* MachineLearningRequestsTypeId = "{B65151FE-3588-432A-A0EE-1DB5BF5147CA}";
} // namespace MachineLearning
