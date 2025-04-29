/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/std/string/string.h>
#include <Lidar/PointCloudMessageBuilder.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ROS2Sensors
{
    PointCloud2MessageBuilder::PointCloud2MessageBuilder(
        const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t count)
    {
        sensor_msgs::msg::PointCloud2 message{};
        message.header.frame_id = frameId.data();
        message.header.stamp = timeStamp;
        message.height = 1;
        message.width = count;

        m_message = message;
    }

    PointCloud2MessageBuilder& PointCloud2MessageBuilder::AddField(const char* name, uint8_t dataType, size_t count)
    {
        sensor_msgs::msg::PointField point_field;
        point_field.name = name;
        point_field.count = count;
        point_field.datatype = dataType;
        point_field.offset = m_offset;
        m_message.fields.push_back(point_field);

        m_offset += point_field.count * sizeOfPointField(dataType);
        return *this;
    }

    sensor_msgs::msg::PointCloud2 PointCloud2MessageBuilder::Get()
    {
        m_message.point_step = m_offset;
        m_message.row_step = m_message.width * m_message.point_step;
        m_message.data.resize(m_message.row_step);

        return m_message;
    }
} // namespace ROS2Sensors
