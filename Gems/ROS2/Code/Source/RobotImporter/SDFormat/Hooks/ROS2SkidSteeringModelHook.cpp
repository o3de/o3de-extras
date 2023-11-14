/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <VehicleDynamics/ModelComponents/SkidSteeringModelComponent.h>

#include <sdf/Plugin.hh>

namespace ROS2::SDFormat
{
    // ROS1             ROS2
    // odometryFrame	odometry_frame
    // updateRate	    update_rate
    // torque	        max_wheel_torque
    // covariance_x	    covariance_x
    // covariance_y	    covariance_y
    // covariance_yaw	covariance_yaw
    // commandTopic	    <ros><remapping>cmd_vel:=custom_cmd_vel</remapping></ros>
    // odometryTopic	<ros><remapping>odom:=custom_odom</remapping></ros>
    // robotBaseFrame	robot_base_frame
    // wheelSeparation	wheel_separation
    // wheelDiameter	wheel_diameter
    // broadcastTF	    publish_wheel_tf / publish_odom_tf
    // leftFrontJoint	1st left_joint
    // rightFrontJoint	1st right_joint
    // leftRearJoint	2nd left_joint
    // rightRearJoint	2nd right_joint

    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2SkidSteeringModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_skid_steer_drive.so" };

        importerHook.m_sdfPluginToComponentCallback =
            []([[maybe_unused]] AZ::Entity& entity,
               [[maybe_unused]] const sdf::Plugin& sdfPlugin) -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            // TODO: print some text for debug -> this should be fixed with a real code
            AZ_Error("ROS2SkidSteeringModel", false, "Hello, is it me you are looking for?");

            return AZ::Success();
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
