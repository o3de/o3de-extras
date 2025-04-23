
#pragma once

namespace ROS2RobotImporter
{
    // System Component TypeIds
    inline constexpr const char* ROS2RobotImporterSystemComponentTypeId = "{BA41DABD-73BD-4D57-84F5-C7573CF306C6}";
    inline constexpr const char* ROS2RobotImporterEditorSystemComponentTypeId = "{8E64EEB9-7531-432E-9CE4-5A4EE27E9427}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2RobotImporterModuleInterfaceTypeId = "{A4047EF8-273E-40B6-A7B0-D4ACB0EEE4DB}";
    inline constexpr const char* ROS2RobotImporterModuleTypeId = "{4A7966DE-7534-4149-9F56-0839F61A8D5D}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2RobotImporterEditorModuleTypeId = ROS2RobotImporterModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2RobotImporterRequestsTypeId = "{3963260F-404B-439A-B451-F4184386A1A7}";
} // namespace ROS2RobotImporter
