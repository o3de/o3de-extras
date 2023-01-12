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
#include <rclcpp/qos.hpp>

namespace ROS2
{
    //! A wrapper for rclcpp::QoS (Quality of Service for DDS) with reflection.
    //! @see <a href="https://design.ros2.org/articles/qos.html">Quality of Service policies</a>.
    struct QoS
    {
    public:
        AZ_TYPE_INFO(QoS, "{46692EA4-EA4C-495E-AD3C-426EAB8954D3}");
        QoS(const rclcpp::QoS& qos = rclcpp::QoS(rmw_qos_profile_default.depth));
        static void Reflect(AZ::ReflectContext* context);

        //! Convert and return a rclcpp::QoS instance based on member values (Editor combos selection).
        //! @return a <a href="https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1QoS.html">ROS2 QoS struct</a>.
        rclcpp::QoS GetQoS() const;

    private:
        AZ::Crc32 OnQoSSelected() const;

        rclcpp::ReliabilityPolicy m_reliabilityPolicy;
        rclcpp::DurabilityPolicy m_durabilityPolicy;
        uint32_t m_depth;
    };
} // namespace ROS2
