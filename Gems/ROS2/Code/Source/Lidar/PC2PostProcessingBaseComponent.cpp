/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Lidar/PC2PostProcessingBaseComponent.h>

namespace ROS2
{
    void PC2PostProcessingBaseComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PC2PostProcessingBaseComponent, AZ::Component>()->Version(0)->Field(
                "Priority", &PC2PostProcessingBaseComponent::m_priority);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PC2PostProcessingBaseComponent>("Point Cloud 2 Post Processing Base Component", "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PC2PostProcessingBaseComponent::m_priority,
                        "Priority",
                        "Post processing priority. Higher priority post processing components are applied first.");
            }
        }
    }

    void PC2PostProcessingBaseComponent::Activate()
    {
        PC2PostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void PC2PostProcessingBaseComponent::Deactivate()
    {
        PC2PostProcessingRequestBus::Handler::BusDisconnect();
    }

    void PC2PostProcessingBaseComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2LidarSensor"));
    }

    AZ::u8 PC2PostProcessingBaseComponent::GetPriority() const
    {
        return m_priority;
    }
} // namespace ROS2
