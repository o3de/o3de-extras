/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LidarSegmentationClassConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void LidarSegmentationClassConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LidarSegmentationClassConfiguration>()
                ->Version(2)
                ->Field("className", &LidarSegmentationClassConfiguration::m_className)
                ->Field("classId", &LidarSegmentationClassConfiguration::m_classId)
                ->Field("classColor", &LidarSegmentationClassConfiguration::m_classColor);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<LidarSegmentationClassConfiguration>("Lidar Segmentation Class Configuration", "Lidar Segmentation Class configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LidarSegmentationClassConfiguration::m_className,
                        "Class Name",
                        "Name of the class")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true)
                    ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &LidarSegmentationClassConfiguration::m_classId,
                                "Class Id",
                            "Id of the class")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true)
                    ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &LidarSegmentationClassConfiguration::m_classColor,
                            "Class Color",
                            "Color of the class")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true);

            }
        }
    }
} // namespace ROS2
