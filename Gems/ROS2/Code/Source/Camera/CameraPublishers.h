/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "CameraSensorDescription.h"

#include <AzCore/std/containers/unordered_map.h>

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace ROS2
{
    //! Handles all the ROS publishing related to a single camera.
    //! This includes 1-2 CameraInfo topics as well as 1-2 Image topics.
    class CameraPublishers
    {
    public:
        //! ROS2 image publisher type.
        using ImagePublisherPtrType = std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>;

        //! ROS2 camera sensor publisher type.
        using CameraInfoPublisherPtrType = std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>;

        CameraPublishers(const CameraSensorDescription& cameraDescription);

        ImagePublisherPtrType GetImagePublisher(CameraSensorDescription::CameraChannelType type);
        CameraInfoPublisherPtrType GetInfoPublisher(CameraSensorDescription::CameraChannelType type);

    private:
        AZStd::unordered_map<CameraSensorDescription::CameraChannelType, ImagePublisherPtrType> m_imagePublishers;
        AZStd::unordered_map<CameraSensorDescription::CameraChannelType, CameraInfoPublisherPtrType> m_infoPublishers;
    };
} // namespace ROS2
