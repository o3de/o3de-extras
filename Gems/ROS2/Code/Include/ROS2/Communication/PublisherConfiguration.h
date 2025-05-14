/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2TypeIds.h>
#include <rclcpp/publisher.hpp>

namespace ROS2
{
    struct PublisherConfiguration
    {
        AZ_TYPE_INFO(PublisherConfiguration, PublisherConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        bool m_publish = true; //!< A switch controlling whether publishing happens.
        TopicConfiguration m_topicConfiguration; //!< Configuration of the published topic.
        float m_frequency = 10.0f; //!< Frequency of the published topic (Hz)
    };
} // namespace ROS2
