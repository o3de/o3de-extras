/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SupportedTags.h"

namespace ROS2::SDFormat
{
    AZStd::unordered_set<AZStd::string> SupportedTags::GetSupportedTags(const sdf::SensorType& sensorType)
    {
        switch (sensorType)
        {
        case sdf::SensorType::CAMERA:
        case sdf::SensorType::DEPTH_CAMERA:
        case sdf::SensorType::RGBD_CAMERA:
            return AZStd::unordered_set<AZStd::string>{
                ">update_rate", ">camera>horizontal_fov", ">camera>image>width", ">camera>image>height"
            };
        default:
            AZ_Warning("AddSensor", false, "Unsupported sensor type, %d", sensorType);
            break;
        };

        return AZStd::unordered_set<AZStd::string>();
    }
} // namespace ROS2::SDFormat
