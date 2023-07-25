/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <sdf/sdf.hh>

namespace ROS2::SDFormat
{
    //! Populates a given entity with all the contents of the <sensor> tag in robot description in SDFormat.
    //! In SDFormat, sensors are specified as children of link or joint.
    class SensorsMaker
    {
    public:
        //! Add a sensor to a link entity and set it accordingly.
        //! @param entityId A non-active entity which will be affected.
        //! @param link A link in parsed SDF model which could hold information about sensor to be made.
        //! @returns created components Id or string with fail
        void AddSensors(AZ::EntityId entityId, const sdf::Link* link) const;

        //! Add a sensor to a joint entity and set it accordingly.
        //! @param entityId A non-active entity which will be affected.
        //! @param joint A joint in parsed SDF model which could hold information about sensor to be made.
        //! @returns created components Id or string with fail
        void AddSensors(AZ::EntityId entityId, const sdf::Joint* joint) const;
    };
} // namespace ROS2::SDFormat
