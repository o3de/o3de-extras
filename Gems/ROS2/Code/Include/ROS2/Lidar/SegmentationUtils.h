/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>

namespace ROS2::SegmentationUtils
{
    //! Returns the segmentation class ID of the entity with provided ID.
    //! Entity's class ID is fetched using the Tag component (@see Tag).
    //! If this entity has a tag with a name that matches an existing
    //! segmentation class (configured through the Class Segmentation component),
    //! the ID of this class is returned. Otherwise, the Unknown Class ID is returned.
    //! @param entityId ID of the entity for which a class ID is to be fetched.
    //! @return Class ID of the entity.
    [[nodiscard]] uint8_t FetchClassIdForEntity(AZ::EntityId entityId);
} // namespace ROS2::SegmentationUtils
