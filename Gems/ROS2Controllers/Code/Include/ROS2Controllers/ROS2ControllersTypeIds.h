/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

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

    // Editor Interface TypeIds
    inline constexpr const char* ROS2ControllersEditorRequestsTypeId = "{2E0AB230-743E-4BCA-8194-D1BC7C9341EA}";

    // Sensors TypeIds
    inline constexpr const char* ROS2WheelOdometryComponentTypeId = "{9BDB8C23-AC76-4C25-8D35-37AAA9F43FAC}";
    inline constexpr const char* ROS2OdometrySensorComponentTypeId = "{61387448-63AA-4563-AF87-60C72B05B863}";
    inline constexpr const char* ROS2OdometryCovarianceTypeId = "{DB015832-F621-11ED-B67E-0242AC120002}";
    inline constexpr const char* WheelOdometrySensorConfigurationTypeId = "{9DC58D89-E674-4D7F-9EA9-AFE3AE7FD2EB}";

    // Controllers Components TypeIds
    inline constexpr const char* FingerGripperComponentTypeId = "{AE5F8EC2-26EE-11EE-BE56-0242AC120002}";
    inline constexpr const char* GripperActionServerComponentTypeId = "{6A4417AC-1D85-4AB0-A116-1E77D40FC816}";
    inline constexpr const char* VacuumGripperComponentTypeId = "{A29EB4FA-0F6F-11EE-BE56-0242AC120002}";
    inline constexpr const char* JointsArticulationControllerComponentTypeId = "{243E9F07-5F84-4F83-9E6D-D1DA04D7CEF9}";
    inline constexpr const char* JointsPIDControllerComponentTypeId = "{41A31EDB-90B0-412E-BBFA-D35D45546A8E}";
    inline constexpr const char* JointsManipulationComponentTypeId = "{3DA9ABFC-0028-4E3E-8D04-4E4440D2E319}";
    inline constexpr const char* JointsManipulationEditorComponentTypeId = "{BF2F77FD-92FB-4730-92C7-DDEE54F508BF}";
    inline constexpr const char* JointsPositionsComponentTypeId = "{21335907-767E-4B2C-81C0-6F0B410B6D87}";
    inline constexpr const char* JointsPositionsEditorComponentTypeId = "{6FEF3251-F910-4BF7-A4E6-1CF4F73DAAf8}";
    inline constexpr const char* JointsTrajectoryComponentTypeId = "{429DE04C-6B6D-4B2D-9D6C-3681F23CBF90}";

} // namespace ROS2Controllers
