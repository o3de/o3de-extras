/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Manipulation/JointInfo.h>

namespace ROS2
{
    //! Interface for controllers that execute the simple movement between two positions one step at a time.
    class JointsPositionControllerRequests : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Whether the implementing controller component is based on articulations.
        //! @return true if the controller works with articulations, false otherwise.
        virtual bool SupportsArticulation()
        {
            return false;
        };

        //! Whether the implementing controller component is based on articulations.
        //! @return true if the controller works with classic joints, false otherwise.
        virtual bool SupportsClassicJoints()
        {
            return false;
        };

        //! Control a joint through specification of its target position.
        //! @param jointName name of the joint to move.
        //! @param joint specification of the joint.
        //! @param currentPosition current position of the joint.
        //! @param targetPosition target position of the joint.
        //! @param deltaTime how much time elapsed in simulation the movement should represent.
        //! @return nothing on success, error message if the command cannot be realize due to controller or entity configuration.
        virtual AZ::Outcome<void, AZStd::string> PositionControl(
            const AZStd::string& jointName,
            JointInfo joint,
            JointPosition currentPosition,
            JointPosition targetPosition,
            float deltaTime) = 0;
    };
    using JointsPositionControllerRequestBus = AZ::EBus<JointsPositionControllerRequests>;
} // namespace ROS2
