
#pragma once

namespace ROS2Controllers
{
    // System Component TypeIds
    inline constexpr const char* ROS2ControllersSystemComponentTypeId = "{D9A7C3B8-383C-4225-B529-1DAF1B2720F0}";
    inline constexpr const char* ROS2ControllersEditorSystemComponentTypeId = "{F72C1A7F-EC8F-4503-B353-B8D4A41E4E4E}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2ControllersModuleInterfaceTypeId = "{CFEFB216-071E-4732-8871-5E10E6064AB3}";
    inline constexpr const char* ROS2ControllersModuleTypeId = "{B5535E60-9AB5-46E0-97E1-3375DAFA2DC9}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2ControllersEditorModuleTypeId = ROS2ControllersModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2ControllersRequestsTypeId = "{2E0AB230-743E-4BCA-8194-D1BC7C9341EA}";
} // namespace ROS2Controllers
