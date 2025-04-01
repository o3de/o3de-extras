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
#include <AzCore/Interface/Interface.h>

namespace ROS2
{
    //! The interface allows to control gripper components through GripperCommand actions.
    //! It is a bus that allows communication between ROS2 GripperCommand Action server with the particular implementation of the gripper.
    //! It encapsulates GripperCommand commands and getters that allow to build the feedback and result messages.
    //! @see <a href="https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/action/GripperCommand.action">GripperCommand</a>
    class GripperRequests : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        //! Send new command to the gripper.
        //! @param position position of the gripper (as a gap size in meters) to be set, for vacuum/electromagnetic gripper it is 0 or 1.
        //! @param maxEffort maximum effort of the gripper to be set in Newtons.
        //! @return Nothing on success, error message if failed.
        virtual AZ::Outcome<void, AZStd::string> GripperCommand(float position, float maxEffort) = 0;

        //! Cancel the current command to the gripper.
        //! @return Nothing on success, error message if failed.
        virtual AZ::Outcome<void, AZStd::string> CancelGripperCommand() = 0;

        //! Get the current position of the gripper.
        //! @return Position of the gripper.
        virtual float GetGripperPosition() const = 0;

        //! Get the current effort of the gripper.
        //! @return Position (gap size in meters) of the gripper.
        virtual float GetGripperEffort() const = 0;

        //! Get if the gripper is not moving (stalled).
        virtual bool IsGripperNotMoving() const = 0;

        //! Get if the gripper reached the set with GripperCommand position.
        virtual bool HasGripperReachedGoal() const = 0;

        //! Get if the gripper command has been cancelled.
        virtual bool HasGripperCommandBeenCancelled() const = 0;
    };

    using GripperRequestBus = AZ::EBus<GripperRequests>;
} // namespace ROS2
