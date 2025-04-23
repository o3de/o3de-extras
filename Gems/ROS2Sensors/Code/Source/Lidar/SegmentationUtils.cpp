/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <ROS2Sensors/Lidar/ClassSegmentationBus.h>
#include <ROS2Sensors/Lidar/SegmentationUtils.h>

namespace ROS2::SegmentationUtils
{
    uint8_t FetchClassIdForEntity(AZ::EntityId entityId)
    {
        AZStd::optional<uint8_t> classId;

        LmbrCentral::Tags entityTags;
        LmbrCentral::TagComponentRequestBus::EventResult(entityTags, entityId, &LmbrCentral::TagComponentRequests::GetTags);
        auto* segmentationInterface = ClassSegmentationInterface::Get();
        if (!segmentationInterface)
        {
            return UnknownClassId;
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
                "Entity with ID: %s has no class tag. Assigning unknown class ID: %u",
                entityId.ToString().c_str(),
                UnknownClassId);
        }

        return classId.value_or(UnknownClassId);
    }
} // namespace ROS2::SegmentationUtils
