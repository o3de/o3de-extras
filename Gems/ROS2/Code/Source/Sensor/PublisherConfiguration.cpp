/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include "PublisherConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    /// A struct used to create and configure publishers
    void PublisherConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PublisherConfiguration>()
                ->Version(1)
                ->Field("Type", &PublisherConfiguration::m_type)
                ->Field("Topic", &PublisherConfiguration::m_topic)
                // TODO - also enable QoS
                ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<PublisherConfiguration>("Publisher configuration", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PublisherConfiguration::m_type, "Type", "Type of topic messages")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PublisherConfiguration::m_topic, "Topic", "Topic with no namespace")
                    ;
            }
        }
    };
}  // namespace ROS2

