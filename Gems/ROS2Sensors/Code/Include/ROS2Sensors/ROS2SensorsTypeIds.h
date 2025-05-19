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

    // ROS2Sensors Interface TypeIds
    inline constexpr const char* ImuConfigurationRequestTypeId = "{8D426B46-DF12-419A-96FB-4E2834567ECD}";
    inline constexpr const char* SensorConfigurationTypeId = "{4755363D-0B5A-42D7-BBEF-152D87BA10D7}";
    inline constexpr const char* SensorConfigurationRequestTypeId = "{01904EAB-FA33-7487-B634-E3F8361EB5FB}";
    inline constexpr const char* ROS2SensorComponentBaseTypeId = "{2DF9A652-DF5D-43B1-932F-B6A838E36E97}";

    // System Sensor Components TypeIds
    inline constexpr const char* ROS2SystemCameraComponentTypeId = "{B4665D39-78FD-40DE-8518-2F6BD345A831}";
    inline constexpr const char* ROS2EditorCameraSystemComponentTypeId = "{407F51C0-92C9-11EE-B9D1-0242AC120002}";
    inline constexpr const char* LidarRegistrarSystemComponentTypeId = "{78CBA3F1-DB2C-46DE-9C3D-C40DD72F2F1E}";
    inline constexpr const char* LidarRegistrarEditorSystemComponentTypeId = "{7F11B599-5ACE-4498-A9A4-AD280C92BACC}";

    // Lidar API TypeIds
    inline constexpr const char* ClassSegmentationBusTypeId = "{69B4109E-25FF-482F-B92E-F19CDF06BCE2}";
    inline constexpr const char* SegmentationClassConfigurationTypeId = "{E46E75F4-1E0E-48CA-A22F-43AFC8F25133}";
    inline constexpr const char* LidarRaycasterBusTypeId = "{253A02C8-B6CB-493C-B16F-012CCF9DB226}";
    inline constexpr const char* LidarRegistrarBusTypeId = "{22030DC7-A1DB-43BD-B748-0FB9EC43CE2E}";
    inline constexpr const char* LidarSystemBusTypeId = "{007871D1-2783-4382-977B-558F436C54A5}";

    // Sensor Configurations TypeIds
    inline constexpr const char* CameraSensorConfigurationTypeId = "{386A2640-442B-473D-BC2A-665D049D7EF5}";
    inline constexpr const char* ImuSensorConfigurationTypeId = "{6788E84F-B985-4413-8E2B-46FBFB667C95}";
    inline constexpr const char* LidarTemplateTypeId = "{9E9EF583-733D-4450-BBA0-ADD4D1BEFBF2}";
    inline constexpr const char* LidarTemplateNoiseParametersTypeId = "{58C007AD-320F-49DF-BC20-6419159EE176}";
    inline constexpr const char* ROS2OdometryCovarianceTypeId = "{DB015832-F621-11ED-B67E-0242AC120002}";
    inline constexpr const char* ROS2WheelOdometryConfigurationTypeId = "{9DC58D89-E674-4D7F-9EA9-AFE3AE7FD2EB}";

    // Sensor Components TypeIds
    inline constexpr const char* ROS2CameraSensorComponentTypeId = "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}";
    inline constexpr const char* ROS2ContactSensorComponentTypeId = "{91272E66-C9F1-4AA2-A9D5-98EAA4EF4E9A}";
    inline constexpr const char* ROS2GNSSSensorComponentTypeId = "{55B4A299-7FA3-496A-88F0-764C75B0E9A7}";
    inline constexpr const char* ROS2ImuSensorComponentTypeId = "{502A955E-7742-4E23-AD77-5E4063739DCA}";
    inline constexpr const char* ROS2LidarSensorComponentTypeId = "{502A955F-7742-4E23-AD77-5E4063739DCA}";
    inline constexpr const char* ROS2Lidar2DSensorComponentTypeId = "{F4C2D970-1D69-40F2-9D4D-B52DCFDD2704}";
    inline constexpr const char* ROS2WheelOdometryComponentTypeId = "{9BDB8C23-AC76-4C25-8D35-37AAA9F43FAC}";
    inline constexpr const char* ROS2OdometrySensorComponent = "{61387448-63AA-4563-AF87-60C72B05B863}";

    // Sensors Components Tools TypeIds
    inline constexpr const char* ClassSegmentationConfigurationComponentTypeId = "{BAB1EA0C-7456-40EA-BC1E-71697137C27C}";
    inline constexpr const char* ROS2ImageEncodingConversionComponentTypeId = "{12449810-D179-44F1-8F72-22D8D3FA4460}";
    inline constexpr const char* EncodingConversionTypeId = "{DB361ADC-B339-4A4E-A10B-C6BF6791EDA6}";
} // namespace ROS2Sensors
