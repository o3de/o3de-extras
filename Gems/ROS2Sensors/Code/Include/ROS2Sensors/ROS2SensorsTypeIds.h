/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace ROS2Sensors
{
    // System Component TypeIds
    inline constexpr const char* ROS2SensorsSystemComponentTypeId = "{ECA6DB16-1F77-4098-956E-F1463B95AE15}";
    inline constexpr const char* ROS2SensorsEditorSystemComponentTypeId = "{F6A1AE53-0ACE-4425-AABB-C338A74CA6F1}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2SensorsModuleInterfaceTypeId = "{2A5B29DE-6056-4553-8C50-F7DDC8F1D4D0}";
    inline constexpr const char* ROS2SensorsModuleTypeId = "{5607AC61-7364-4690-9DF6-6265EAC0C1DB}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2SensorsEditorModuleTypeId = ROS2SensorsModuleTypeId;

    // Sensor Components Base TypeIds
    inline constexpr const char* ROS2SensorComponentBaseTypeId = "{A0C2D1B4-8E5F-4E7C-8A3D-6F9B5F1A0E7D}";

    // System Sensor Components TypeIds
    inline constexpr const char* ROS2SystemCameraComponentTypeId = "{B4665D39-78FD-40DE-8518-2F6BD345A831}";
    inline constexpr const char* ROS2EditorCameraSystemComponentTypeId = "{407F51C0-92C9-11EE-B9D1-0242AC120002}";
    inline constexpr const char* LidarRegistrarSystemComponentTypeId = "{78CBA3F1-DB2C-46DE-9C3D-C40DD72F2F1E}";
    inline constexpr const char* LidarRegistrarEditorSystemComponentTypeId = "{7F11B599-5ACE-4498-A9A4-AD280C92BACC}";

    // Sensor Components TypeIds
    inline constexpr const char* ROS2CameraSensorComponentTypeId = "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}";
    inline constexpr const char* ROS2ContactSensorComponentTypeId = "{91272e66-c9f1-4aa2-a9d5-98eaa4ef4e9a}";
    inline constexpr const char* ROS2GNSSSensorComponentTypeId = "{55B4A299-7FA3-496A-88F0-764C75B0E9A7}";
    inline constexpr const char* ROS2ImuSensorComponentTypeId = "{502A955E-7742-4E23-AD77-5E4063739DCA}";
    inline constexpr const char* ROS2LidarSensorComponentTypeId = "{502A955F-7742-4E23-AD77-5E4063739DCA}";
    inline constexpr const char* ROS2Lidar2DSensorComponentTypeId = "{F4C2D970-1D69-40F2-9D4D-B52DCFDD2704}";
    inline constexpr const char* ROS2WheelOdometryComponentTypeId = "{9bdb8c23-ac76-4c25-8d35-37aaa9f43fac}";
    inline constexpr const char* ROS2OdometrySensorComponent = "{61387448-63AA-4563-AF87-60C72B05B863}";

    // Sensors Components Tools TypeIds
    inline constexpr const char* ClassSegmentationConfigurationComponentTypeId = "{BAB1EA0C-7456-40EA-BC1E-71697137c27c}";
    inline constexpr const char* ROS2ImageEncodingConversionComponentTypeId = "{12449810-D179-44F1-8F72-22D8d3fa4460}";
    inline constexpr const char* EncodingConversionTypeId = "{DB361ADC-B339-4A4E-A10B-C6BF6791EDA6}";
} // namespace ROS2Sensors
