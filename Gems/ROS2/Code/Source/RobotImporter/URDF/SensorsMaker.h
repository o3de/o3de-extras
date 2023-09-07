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

#include <sdf/sdf.hh>

namespace ROS2
{
    //! Populates a given entity with all the contents of the <sensor> tag in robot description of Gazebo.
    //! In SDFormat, sensors are specified as children of link of joint.
    class SensorsMaker
    {
    public:
        //! Adds a Gazebo sensor to an entity and sets it accordingly
        //! @param model A parsed SDF model which could hold information about sensor to be made.
        //! @param link A parsed SDF tree link node used to identify link being currently processed.
        //! @param entityId A non-active entity which will be affected.
        //! @returns created components Id or string with fail
        void AddSensors(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId) const;
    };
} // namespace ROS2
