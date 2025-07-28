/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/ROS2TypeIds.h>
#include <rclcpp/qos.hpp>

namespace ROS2
{
    //! A wrapper for rclcpp::QoS (Quality of Service for DDS) with reflection.
    //! @see <a href="https://design.ros2.org/articles/qos.html">Quality of Service policies</a>.
    struct QoS
    {
    public:
        AZ_TYPE_INFO(QoS, QoSTypeId);

        QoS(const rclcpp::QoS& qos = rclcpp::QoS(rmw_qos_profile_default.depth))
            : m_reliabilityPolicy(qos.reliability())
            , m_durabilityPolicy(qos.durability())
            , m_depth(qos.depth())
        {
        }

        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<QoS>()
                    ->Version(1)
                    ->Field("Reliability", &QoS::m_reliabilityPolicy)
                    ->Field("Durability", &QoS::m_durabilityPolicy)
                    ->Field("Depth", &QoS::m_depth);

                if (AZ::EditContext* ec = serializeContext->GetEditContext())
                {
                    ec->Class<QoS>("Quality of Service configuration", "")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->DataElement(
                            AZ::Edit::UIHandlers::ComboBox,
                            &QoS::m_reliabilityPolicy,
                            "Reliability policy",
                            "Determines DDS reliability policy for the publisher")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &QoS::OnQoSSelected)
                        ->EnumAttribute(rclcpp::ReliabilityPolicy::Reliable, "Reliable")
                        ->EnumAttribute(rclcpp::ReliabilityPolicy::BestEffort, "Best Effort")
                        ->DataElement(
                            AZ::Edit::UIHandlers::ComboBox,
                            &QoS::m_durabilityPolicy,
                            "Durability policy",
                            "Determines DDS durability policy for the publisher")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &QoS::OnQoSSelected)
                        ->EnumAttribute(rclcpp::DurabilityPolicy::Volatile, "Volatile")
                        ->EnumAttribute(rclcpp::DurabilityPolicy::TransientLocal, "Transient Local")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &QoS::m_depth, "History depth", "Determines DDS publisher queue size");
                }
            }
        };

        //! Convert and return a rclcpp::QoS instance based on member values (Editor combos selection).
        //! @return a <a href="https://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1QoS.html">ROS2 QoS struct</a>.
        rclcpp::QoS GetQoS() const
        {
            rclcpp::QoS qos(m_depth);
            return qos.reliability(m_reliabilityPolicy).durability(m_durabilityPolicy);
        }

    private:
        AZ::Crc32 OnQoSSelected()
        {
            return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
        }

        rclcpp::ReliabilityPolicy m_reliabilityPolicy;
        rclcpp::DurabilityPolicy m_durabilityPolicy;
        uint32_t m_depth;
    };
} // namespace ROS2
