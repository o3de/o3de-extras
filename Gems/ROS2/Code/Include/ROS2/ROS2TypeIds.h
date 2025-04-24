
#pragma once

namespace ROS2
{
    // System Component TypeIds
    inline constexpr const char* ROS2SystemComponentTypeId = "{3846256B-CF12-4CF0-9FF7-612642EC8640}";
    inline constexpr const char* ROS2EditorSystemComponentTypeId = "{349883C1-C66E-45F5-90D7-884565FCFA7E}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2ModuleInterfaceTypeId = "{6F7D19CA-122B-4416-BAF6-828530BA30F9}";
    inline constexpr const char* ROS2ModuleTypeId = "{7AD345EB-3463-41BC-9AAF-68DAF8AA396A}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2EditorModuleTypeId = ROS2ModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2RequestsTypeId = "{A02A14D0-3CBD-4513-98C5-944D8B0977BE}";
} // namespace ROS2
