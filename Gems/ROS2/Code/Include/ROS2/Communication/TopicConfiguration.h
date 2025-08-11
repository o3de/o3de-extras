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
#include <AzCore/std/string/string.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    //! A structure for a single ROS2 topic, a part of publisher or subscriber configuration.
    struct TopicConfiguration
    {
    public:
        AZ_TYPE_INFO(TopicConfiguration, TopicConfigurationTypeId);

        TopicConfiguration() = default;
        TopicConfiguration(const QoS& qos)
            : m_qos(qos)
        {
        }

        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<TopicConfiguration>()
                    ->Version(1)
                    ->Field("Type", &TopicConfiguration::m_type)
                    ->Field("Topic", &TopicConfiguration::m_topic)
                    ->Field("QoS", &TopicConfiguration::m_qos);

                if (AZ::EditContext* ec = serializeContext->GetEditContext())
                {
                    ec->Class<TopicConfiguration>("Publisher configuration", "")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &TopicConfiguration::m_type, "Type", "Type of topic messages")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &TopicConfiguration::m_topic, "Topic", "Topic with no namespace")
                        ->Attribute(AZ::Edit::Attributes::ChangeValidate, &TopicConfiguration::ValidateTopicField)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &TopicConfiguration::m_qos, "QoS", "Quality of Service");
                }
            }
        }

        AZStd::string m_type = "std_msgs::msg::Empty"; //!< descriptive topic type for identification.
        AZStd::string m_topic = "default_topic"; //!< Topic to publish. Final topic will have a namespace added.

        //! Get topic QoS (Quality of Service) settings.
        //! @see ROS2::QoS.
        rclcpp::QoS GetQoS() const
        {
            return m_qos.GetQoS();
        }

    private:
        //! Helper function for the UI to validate topic names.
        AZ::Outcome<void, AZStd::string> ValidateTopicField(void* newValue, const AZ::Uuid& valueType)
        {
            AZ::Outcome<void, AZStd::string> outcome;
            ROS2NamesRequestBus::BroadcastResult(outcome, &ROS2NamesRequests::ValidateTopicField, newValue, valueType);

            return outcome;
        }

        QoS m_qos = rclcpp::SensorDataQoS();
    };
} // namespace ROS2
