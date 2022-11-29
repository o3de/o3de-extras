/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "QoS.h"
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! A structure for a single ROS2 topic, a part of publisher or subscriber configuration.
    struct TopicConfiguration
    {
    public:
        AZ_TYPE_INFO(TopicConfiguration, "{7535D58F-5284-4657-A799-1F69D3F5AA42}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_type = "std_msgs::msg::Empty"; //!< descriptive topic type for identification.
        AZStd::string m_topic = "default_topic"; //!< Topic to publish. Final topic will have a namespace added.

        //! Get topic QoS (Quality of Service) settings.
        //! @see ROS2::QoS.
        rclcpp::QoS GetQoS() const
        {
            return m_qos.GetQoS();
        }

    private:
        QoS m_qos = rclcpp::SensorDataQoS();
    };
} // namespace ROS2
