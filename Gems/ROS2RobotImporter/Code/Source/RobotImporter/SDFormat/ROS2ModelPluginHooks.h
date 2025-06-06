/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2RobotImporter/SDFormatModelPluginImporterHook.h>

namespace ROS2RobotImporter::SDFormat::ROS2ModelPluginHooks
{
    // temporarily disable import hooks for sensors and models for https://github.com/o3de/sig-simulation/pull/96

    // //! Get a mapping of SDFormat skid_steer_drive and diff_drive plugins into O3DE components.
    // ModelPluginImporterHook ROS2SkidSteeringModel();

    //! Get a mapping of SDFormat ackermann_drive plugin into O3DE components.
    ModelPluginImporterHook ROS2AckermannModel();

    // //! Get a mapping of SDFormat joint_state_publisher plugin into O3DE components.
    // ModelPluginImporterHook ROS2JointStatePublisherModel();

    //  //! Get a mapping of SDFormat joint_pose_trajectory plugin into O3DE components.
    // ModelPluginImporterHook ROS2JointPoseTrajectoryModel();

} // namespace ROS2RobotImporter::SDFormat::ROS2ModelPluginHooks
