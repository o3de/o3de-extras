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
#include <AzCore/std/string/string.h>
#include "QoS/QoS.h"

namespace ROS2
{
    /// A struct used to create and configure publishers
    struct PublisherConfiguration
    {
    public:
        AZ_TYPE_INFO(PublisherConfiguration, "{7F875348-F2F9-404A-841E-D9A749EA4E79}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_type = "std_msgs::msg::Empty";
        AZStd::string m_topic = "default_topic";

        rclcpp::QoS GetQoS() const { return m_qos.GetQoS(); }

    private:
        QoS m_qos = rclcpp::SensorDataQoS();
    };
}  // namespace ROS2

