/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImageFormatConvertComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2ImageFormatConvert::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImageFormatConvert, AZ::Component>()->Version(0)->Field(
                "Priority", &ROS2ImageFormatConvert::m_priority);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2ImageFormatConvert>("Image Format Convert", "Converts image format to a different format")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageFormatConvert::m_priority,
                        "Priority",
                        "Priority of the post processing. The higher the number the later the post processing is applied.")
                    ->Attribute(AZ::Edit::Attributes::Min, CameraPostProcessingRequests::MIN_PRIORITY)
                    ->Attribute(AZ::Edit::Attributes::Max, CameraPostProcessingRequests::MAX_PRIORITY);
            }
        }
    }

    void ROS2ImageFormatConvert::Activate()
    {
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2ImageFormatConvert::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();
    }

    void ROS2ImageFormatConvert::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        AZ_TracePrintf("ROS2ImageFormatConvert", "ApplyPostProcessing");
    }

    AZ::u8 ROS2ImageFormatConvert::GetPriority() const
    {
        return m_priority;
    }

} // namespace ROS2
