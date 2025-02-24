/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

namespace Georeferencing
{
    // System Component TypeIds
    inline constexpr const char* GeoreferencingSystemComponentTypeId = "{41D42FB6-4C76-4400-8E39-68F8AAD4CB55}";
    inline constexpr const char* GeoreferencingEditorSystemComponentTypeId = "{4D7E4975-EE64-4030-BEE0-671BF8CB8EC2}";

    // Module derived classes TypeIds
    inline constexpr const char* GeoreferencingModuleInterfaceTypeId = "{3F81E669-EA46-4C78-8596-040237413B02}";
    inline constexpr const char* GeoreferencingModuleTypeId = "{0D488925-63E0-49BD-90A5-2CE96B7346FE}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* GeoreferencingEditorModuleTypeId = GeoreferencingModuleTypeId;

    inline constexpr const char* WGS84CoordinateTypeId = "{2E9FFFDC-0025-44EE-97B4-B42C0D59CAF9}";
    inline constexpr const char* GeoreferenceRequestsTypeId = "{DD8F754A-D7A8-4DDD-8CBD-AA0F137DBF1D}";
    inline constexpr const char* GeoreferenceConfigurationRequestsTypeId = "{AF78B7C1-2E9E-4955-BC2C-AB1F45C8186F}";
    inline constexpr const char* GeoReferenceLevelEditorComponentTypeId = "{3A8789E8-3FA1-4A79-B262-02AF16379EAA}";
    inline constexpr const char* GeoReferenceLevelComponentTypeId = "{711E912E-FF60-442F-9010-8503AD802246}";

    inline constexpr const char* GeoReferenceLevelConfigTypeId = "{6C51A9EC-49E8-4344-9E3E-D2627C245FE1}";

} // namespace Georeferencing
