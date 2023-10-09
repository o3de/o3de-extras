/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

namespace ROS2
{
    namespace CameraConstants
    {
        inline constexpr char ImageMessageType[] = "sensor_msgs::msg::Image";
        inline constexpr char DepthImageConfig[] = "Depth Image";
        inline constexpr char ColorImageConfig[] = "Color Image";
        inline constexpr char DepthInfoConfig[] = "Depth Camera Info";
        inline constexpr char ColorInfoConfig[] = "Color Camera Info";
        inline constexpr char CameraInfoMessageType[] = "sensor_msgs::msg::CameraInfo";
    } // namespace CameraConstants
} // namespace ROS2
