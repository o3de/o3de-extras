/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void TopicConfiguration::Reflect(AZ::ReflectContext* context)
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
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateTopicField)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &TopicConfiguration::m_qos, "QoS", "Quality of Service");
            }
        }
    };
} // namespace ROS2
