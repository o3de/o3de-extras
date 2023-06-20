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
   //! Interface allows to control the grippers Components.
   class GripperRequest : public AZ::EBusTraits
   {
   public:
       using BusIdType = AZ::EntityId;
       static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
       static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

       //! Send new command to the gripper.
       //! @param position position of the gripper to be set, for vacuum/electromagnetic gripper it is 0 or 1.
       //! @param maxEffort maximum effort of the gripper to be set
       virtual AZ::Outcome<void, AZStd::string> GripperCommand(float position, float maxEffort) = 0;

       //! Cancel the current command to the gripper.
       virtual AZ::Outcome<void, AZStd::string> CancelGripperCommand() = 0;


       //! Get the current position of the gripper.
       //! @note that for vacuum grippers it is 0 or 1 and it show if vacuum was created and object is attached to the gripper.
       virtual float GetGripperPosition() const = 0;

       //! Get the current effort of the gripper.
       //! @note that for vacuum grippers it is 0 or 1 and it show if vacuum was created and object is attached to the gripper.
       virtual float GetGripperEffort() const = 0;

       //! Get if the gripper is not moving (stalled).
       //! @note that for vacuum grippers it vacuum was created and object is attached to the gripper.
       virtual bool IsGripperNotMoving() const = 0;

       //! Get if the gripper reached the set with GripperCommand position.
       //! @note that for vacuum grippers it vacuum was created and object is attached to the gripper.
       virtual bool IsGripperReachedGoal() const = 0;

   };

   using GripperRequestBus = AZ::EBus<GripperRequest>;
} // namespace ROS2
