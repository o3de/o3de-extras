/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "UrdfParser.h"
#include <AzCore/Component/EntityId.h>

namespace ROS2
{
    //! Populates the entity with contents of the <inertial> tag in robot description.
    class InertialsMaker
    {
    public:
        //! Add zero or one inertial elements to a given entity (depending on link content).
        //! @param inertial A pointer to a parsed SDF inertial structure, might be null.
        //! @param entityId A non-active entity which will be populated according to inertial content.
        void AddInertial(const gz::math::Inertiald& inertial, AZ::EntityId entityId) const;
    };
} // namespace ROS2
