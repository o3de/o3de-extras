/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Communication/QoS.h>

namespace ROS2
{
    QoS::QoS(const rclcpp::QoS& qos)
        : m_reliabilityPolicy(qos.reliability())
        , m_durabilityPolicy(qos.durability())
        , m_depth(qos.depth())
    {
    }

    AZ::Crc32 QoS::OnQoSSelected() const
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    rclcpp::QoS QoS::GetQoS() const
    {
        rclcpp::QoS qos(m_depth);
        return qos.reliability(m_reliabilityPolicy).durability(m_durabilityPolicy);
    }

    void QoS::Reflect(AZ::ReflectContext* context)
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
} // namespace ROS2
