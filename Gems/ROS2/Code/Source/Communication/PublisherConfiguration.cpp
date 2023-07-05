/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void PublisherConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PublisherConfiguration>()
                ->Version(1)
                ->Field("Publish", &PublisherConfiguration::m_publish)
                ->Field("Topic", &PublisherConfiguration::m_topicConfiguration)
                ->Field("Frequency", &PublisherConfiguration::m_frequency);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<PublisherConfiguration>("Publisher", "Configuration of publisher")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &PublisherConfiguration::m_publish, "Publish", "Whether the publisher is on")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &PublisherConfiguration::m_topicConfiguration, "Topic", "Topic configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &PublisherConfiguration::m_frequency, "Frequency (Hz)", "Publishing frequency");
            }
        }
    }
} // namespace ROS2
