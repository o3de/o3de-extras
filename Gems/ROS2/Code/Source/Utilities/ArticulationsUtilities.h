/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/Component/Entity.h>

namespace ROS2::Utils
{
    //! Retrieve root of articulation for given entity. If the entity is not part of an articulation, the invalid entity id is returned.
    //! @param entityId The entity to get the root of.
    //! @return The root of articulation
    AZ::EntityId GetRootOfArticulation(AZ::EntityId entityId);

} // namespace ROS2::Utils
