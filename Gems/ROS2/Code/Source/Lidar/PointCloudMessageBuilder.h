/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <AzCore/std/optional.h>

namespace ROS2
{
    class PointCloud2MessageBuilder
    {
    public:
        PointCloud2MessageBuilder(const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t count);
        PointCloud2MessageBuilder& AddField(const char* name, uint8_t dataType, size_t count = 1);
        sensor_msgs::msg::PointCloud2 Get();

    private:
        size_t m_offset = 0U;
        sensor_msgs::msg::PointCloud2 m_message;
    };
} // namespace ROS2
