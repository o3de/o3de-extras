/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>

namespace ROS2::SDFormat
{
    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2SkidSteeringModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames =
            AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_diff_drive.so", "libgazebo_ros_skid_steer_drive.so" };

        importerHook.m_sdfPluginToComponentCallback = [](AZ::Entity& entity,
                                                         const sdf::Model& sdfSensor) -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            // TODO: print some text for debug -> this should be fixed with a real code
            AZ_Error("ROS2SkidSteeringModel", false, "Hello, is it me you are looking for?");
            return AZ::Success();
        };

        return importerHook;
    }
} // namespace ROS2::SDFormat
