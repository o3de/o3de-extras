/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraSensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void CameraSensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<CameraSensorConfiguration>()
                ->Version(1)
                ->Field("VerticalFieldOfViewDeg", &CameraSensorConfiguration::m_verticalFieldOfViewDeg)
                ->Field("Width", &CameraSensorConfiguration::m_width)
                ->Field("Height", &CameraSensorConfiguration::m_height)
                ->Field("Depth", &CameraSensorConfiguration::m_depthCamera)
                ->Field("Color", &CameraSensorConfiguration::m_colorCamera);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<CameraSensorConfiguration>("Camera Sensor Configuration", "Configuration for image size, FOV and type of images")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &CameraSensorConfiguration::m_verticalFieldOfViewDeg,
                        "Vertical field of view",
                        "Camera's vertical (y axis) field of view in degrees.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 179.99f) // asserted to be less than 180 ([see the link](https://github.com/o3de/o3de-extras/blob/stabilization/2305/Gems/ROS2/Code/Source/Camera/CameraSensor.cpp#L72))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraSensorConfiguration::m_width, "Image width", "Image width")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraSensorConfiguration::m_height, "Image height", "Image height")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraSensorConfiguration::m_colorCamera, "Color Camera", "Color Camera")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &CameraSensorConfiguration::m_depthCamera, "Depth Camera", "Depth Camera");
            }
        }
    }
} // namespace ROS2
