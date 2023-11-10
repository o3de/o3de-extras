/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/RobotImporter/SDFormatModelPluginImporterHook.h>

namespace ROS2::SDFormat
{
    namespace ROS2ModelPluginHooks
    {
        //! Retrieve a model plugin importer hook which is used to map SDFormat model plugin of type skid steering drive into O3DE
        //! components.
        ModelPluginImporterHook ROS2SkidSteeringModel();
    } // namespace ROS2ModelPluginHooks
} // namespace ROS2::SDFormat
