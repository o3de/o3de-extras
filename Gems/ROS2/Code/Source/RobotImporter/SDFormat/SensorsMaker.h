/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>

#include <sdf/Sensor.hh>
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
        void AddSensors(const AZ::EntityId entityId, const sdf::Link* link);

        //! Add a sensor to a joint entity and set it accordingly.
        //! @param entityId A non-active entity which will be affected.
        //! @param joint A joint in parsed SDF model which could hold information about sensor to be made.
        void AddSensors(const AZ::EntityId entityId, const sdf::Joint* joint);

        //! Get log data created while adding sensors
        const AZStd::string& GetLog() const;

        //! Clean log data created while adding sensors
        void ResetLog();

    private:
        void AddSensor(const AZ::EntityId entityId, const sdf::Sensor* sensor);

        AZStd::string m_log;
    };
} // namespace ROS2::SDFormat
