/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Lidar/ClassSegmentationBus.h>
#include <ROS2/Lidar/SegmentationClassConfiguration.h>

namespace ROS2
{
    const SegmentationClassConfiguration SegmentationClassConfiguration::UnknownClass =
        SegmentationClassConfiguration{ "Unknown", UnknownClassId, AZ::Color(1.0f, 1.0f, 1.0f, 1.0f) };
    const SegmentationClassConfiguration SegmentationClassConfiguration::GroundClass =
        SegmentationClassConfiguration{ "Ground", TerrainClassId, AZ::Color(0.5f, 0.25f, 0.0f, 1.0f) };

    void SegmentationClassConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SegmentationClassConfiguration>()
                ->Version(2)
                ->Field("className", &SegmentationClassConfiguration::m_className)
                ->Field("classId", &SegmentationClassConfiguration::m_classId)
                ->Field("classColor", &SegmentationClassConfiguration::m_classColor);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<SegmentationClassConfiguration>(
                      "Lidar Segmentation Class Configuration", "Lidar Segmentation Class configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SegmentationClassConfiguration::m_className, "Class Name", "Name of the class")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &SegmentationClassConfiguration::m_classId, "Class Id", "Id of the class")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &SegmentationClassConfiguration::m_classColor, "Class Color", "Color of the class")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true);
            }
        }
    }
} // namespace ROS2
