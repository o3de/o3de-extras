/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "QoS/QoS.h"
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! A structure for a single ROS2 publisher configuration.
    struct PublisherConfiguration
    {
    public:
        AZ_TYPE_INFO(PublisherConfiguration, "{7F875348-F2F9-404A-841E-D9A749EA4E79}");
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
