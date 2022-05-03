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
    /// A wrapper on rclcpp::QoS (Quality of Service for DDS) with reflection
    struct QoS
    {
    public:
        AZ_RTTI(QoS, "{46692EA4-EA4C-495E-AD3C-426EAB8954D3}");
        QoS(const rclcpp::QoS& qos = rclcpp::QoS(rmw_qos_profile_default.depth));
        virtual ~QoS() = default;
        static void Reflect(AZ::ReflectContext* context);
        rclcpp::QoS GetQoS() const;

    private:
        AZ::Crc32 OnQoSSelected();

        // TODO - only for Editor component
        // Can be extended to also expose history and liveliness
        rclcpp::ReliabilityPolicy m_reliabilityPolicy;
        rclcpp::DurabilityPolicy m_durabilityPolicy;
        uint32_t m_depth;
    };
}  // namespace ROS2
