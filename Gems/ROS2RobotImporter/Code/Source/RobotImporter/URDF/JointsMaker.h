/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "UrdfParser.h"
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Outcome/Outcome.h>

namespace ROS2
{
    //! Populates a given entity with all the contents of the <joint> tag in robot description.
    //! In URDF/SDF, joints are specified between two given links, but in PhysX they are between two Bodies / Colliders.
    class JointsMaker
    {
    public:
        using JointsMakerResult = AZ::Outcome<AZ::ComponentId, AZStd::string>;

        //! Add a joint to an entity and sets it accordingly to sdf::Joint
        //! @param joint Joint data
        //! @param followColliderEntityId A non-active entity which will be populated with Joint components.
        //! @param leadColliderEntityId An entity higher in hierarchy which is connected through the joint with the child entity.
        //! @returns created components Id or string with fail
        JointsMakerResult AddJointComponent(
            const sdf::Joint* joint, AZ::EntityId followColliderEntityId, AZ::EntityId leadColliderEntityId) const;
    };
} // namespace ROS2
