/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <ROS2/Lidar/ClassSegmentationBus.h>
#include <ROS2/Lidar/SegmentationUtils.h>

namespace ROS2::SegmentationUtils
{
    uint8_t FetchClassIdForEntity(AZ::EntityId entityId)
    {
        static constexpr uint8_t DefaultClassID = 0U;
        AZStd::optional<uint8_t> classId;

        LmbrCentral::Tags entityTags;
        LmbrCentral::TagComponentRequestBus::EventResult(entityTags, entityId, &LmbrCentral::TagComponentRequests::GetTags);
        auto* segmentationInterface = ClassSegmentationInterface::Get();
        if (!segmentationInterface)
        {
            AZ_Error(
                __func__,
                false,
                "Segmentation Interface was not accessible. Unable to fetch class ID for entity. Assigning default class ID: %u",
                DefaultClassID);
            return DefaultClassID;
        }

        for (const auto& tag : entityTags)
        {
            AZStd::optional<uint8_t> tagClassId = segmentationInterface->GetClassIdForTag(tag);
            if (tagClassId.has_value())
            {
                if (classId.has_value())
                {
                    AZ_Warning(
                        "EntityManager",
                        false,
                        "Entity with ID: %s has more than one class tag. Assigning first class ID %u",
                        entityId.ToString().c_str(),
                        classId.value());
                }
                else
                {
                    classId = tagClassId.value();
                }
            }
        }

        if (!classId.has_value())
        {
            AZ_Warning(
                "EntityManager",
                false,
                "Entity with ID: %s has no class tag. Assigning default class ID: %u",
                entityId.ToString().c_str(),
                DefaultClassID);
        }

        return classId.value_or(DefaultClassID);
    }
} // namespace ROS2::SegmentationUtils
