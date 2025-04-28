
#pragma once

namespace ROS2
{
    // System Component TypeIds
    inline constexpr const char* ROS2SystemComponentTypeId = "{CB28D486-AFA4-4A9F-A237-AC5EB42E1C87}";
    inline constexpr const char* ROS2EditorSystemComponentTypeId = "{349883C1-C66E-45F5-90D7-884565FCFA7E}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2ModuleInterfaceTypeId = "{8B5567CB-1DE9-49AF-9CD4-9750D4ABCD6B}";
    inline constexpr const char* ROS2ModuleTypeId = "{3DDFC98F-D1CC-4658-BAF8-2CC34A9D39F3}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2EditorModuleTypeId = ROS2ModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2RequestsTypeId = "{A9BDBFF6-E644-430D-8096-CDB53C88E8FC}";
} // namespace ROS2
