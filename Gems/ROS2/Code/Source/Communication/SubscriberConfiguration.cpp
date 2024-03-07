/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Communication/SubscriberConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void SubscriberConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SubscriberConfiguration>()
                ->Version(0)
                ->Field("Subscribe", &SubscriberConfiguration::m_subscribe)
                ->Field("Topic", &SubscriberConfiguration::m_topicConfiguration);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<SubscriberConfiguration>("Subscriber", "Configuration of subscriber")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SubscriberConfiguration::m_subscribe, "Subscribe", "Whether the subscriber is on")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SubscriberConfiguration::m_topicConfiguration, "Topic", "Topic configuration");
            }
        }
    }
} // namespace ROS2
