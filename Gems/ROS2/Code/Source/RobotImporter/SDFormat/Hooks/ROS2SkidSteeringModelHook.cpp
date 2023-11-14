/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.h>
#include <RobotControl/ROS2RobotControlComponent.h>
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
        importerHook.m_pluginNames =
            AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_skid_steer_drive.so", "libgazebo_ros_diff_drive.so" };

        importerHook.m_sdfPluginToComponentCallback =
            []([[maybe_unused]] AZ::Entity& entity,
               [[maybe_unused]] const sdf::Plugin& sdfPlugin) -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            VehicleDynamics::AxleConfiguration frontAxle;
            VehicleDynamics::AxleConfiguration backAxle;

            frontAxle.m_axleTag = "front";
            frontAxle.m_isSteering = true;
            frontAxle.m_isDrive = true;
            frontAxle.m_wheelRadius = 0.11f;
            backAxle.m_axleTag = "back";
            backAxle.m_isDrive = true;
            backAxle.m_wheelRadius = 0.13f;

            VehicleDynamics::VehicleConfiguration vehicleConfiguration;
            vehicleConfiguration.m_axles.emplace_back(AZStd::move(frontAxle));
            vehicleConfiguration.m_axles.emplace_back(AZStd::move(backAxle));
            vehicleConfiguration.m_track = 11.0;
            vehicleConfiguration.m_wheelbase = 1.3;

            VehicleDynamics::SkidSteeringModelLimits modelLimits(11.0f, 13.0f, 15.0f, 17.0f);

            // Create required components
            // Note: ROS2FrameComponent and ROS2RobotControlComponent might be created already
            HooksUtils::CreateComponent<ROS2FrameComponent>(entity);
            HooksUtils::CreateComponent<ROS2RobotControlComponent>(entity);
            HooksUtils::CreateComponent<VehicleDynamics::SkidSteeringModelComponent>(
                entity, vehicleConfiguration, VehicleDynamics::SkidSteeringDriveModel(modelLimits));

            // Create Skid Steering Control Component
            if (HooksUtils::CreateComponent<SkidSteeringControlComponent>(entity))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS2 Skid Steering Control Component"));
            }
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
