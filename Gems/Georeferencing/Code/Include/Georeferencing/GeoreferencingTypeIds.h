
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

} // namespace Georeferencing
