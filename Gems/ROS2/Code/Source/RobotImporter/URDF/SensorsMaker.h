/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/containers/set.h>
#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>

#include <sdf/sdf.hh>

namespace ROS2
{
    //! Populates a given entity with all the contents of the <sensor> tag.
    //! Sensors are specified as children of link or joint in SDFormat.
    class SensorsMaker
    {
    public:
        //! Adds a sensor to an entity and sets it accordingly based on SDFormat description.
        //! @param model A parsed SDF model which could hold information about sensor to be made.
        //! @param link A parsed SDF tree link node used to identify link being currently processed.
        //! @param entityId A non-active entity which will be affected.
        //! @return List containing any entities with sensors that were created.
        AZStd::vector<AZ::EntityId> AddSensors(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId);

        //! Get a reference to collection of status messages (read-only)
        //! @return A reference to set containing status messages.
        const AZStd::set<AZStd::string>& GetStatusMessages() const;

    private:
        AZStd::set<AZStd::string> m_status;

        using SensorHookCallOutcome = AZ::Outcome<void, AZStd::string>;
        SensorHookCallOutcome AddSensor(AZ::EntityId entityId, const sdf::Sensor* sensor, AZStd::vector<AZ::EntityId>& createdEntities);
        SensorHookCallOutcome CallSensorHook(
            AZ::EntityId entityId,
            const sdf::Sensor* sensor,
            const SDFormat::SensorImporterHook* hook,
            AZStd::vector<AZ::EntityId>& createdEntities);
    };
} // namespace ROS2
