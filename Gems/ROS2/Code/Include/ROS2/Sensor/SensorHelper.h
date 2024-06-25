/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>

namespace ROS2
{
    //! Function to check if the component is a sensor component
    //! @param component Component to check
    //! @return True if the component is a sensor component
    bool IsComponentROS2Sensor(const AZ::Component* component);

    //! Function to get Ids of all sensors attached to the entity
    //! @param entityId Id of the entity
    //! @return Vector of EntityComponentIdPair of all sensors attached to the entity
    AZStd::vector<AZ::EntityComponentIdPair> GetSensorsForEntity(const AZ::EntityId& entityId);

    //! Function to get Ids of all sensors attached to the entity
    //! @param entityId Id of the entity
    //! @param sensorType Type of the sensor, see @file ROS2SensorTypesIds.h
    //! @return Vector of EntityComponentIdPair of all sensors attached to the entity
    AZStd::vector<AZ::EntityComponentIdPair> GetSensorsOfType(const AZ::EntityId& entityId, const AZ::Uuid& sensorType);

} // namespace ROS2
