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
#include <rclcpp/publisher.hpp>

namespace ROS2
{
    struct SubscriberConfiguration
    {
        AZ_TYPE_INFO(SubscriberConfiguration, "{b8411e11-12a0-4404-a528-16297d10e664}");
        static void Reflect(AZ::ReflectContext* context);

        bool m_subscribe = true;  //!< A switch controlling whether subscription happens.
        TopicConfiguration m_topicConfiguration; //!< Configuration of the subscribed topic.
    };
} // namespace ROS2
