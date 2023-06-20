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
#include <AzCore/Outcome/Outcome.h>
#include <control_msgs/msg/joint_trajectory.hpp>

namespace ROS2
{
   //! Interface for commanding robotic arm (manipulator) movement.
   class ManipulatorTrajectoryRequests : public AZ::EBusTraits
   {
   public:
       using BusIdType = AZ::EntityId;
       static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

       using TrajectoryGoalPtr = std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal>;
       using TrajectoryResultPtr = std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Result>;

       enum class ManipulatorActionStatus
       {
           Idle,
           Executing,
           Cancelled,
           Succeeded
       };

       virtual AZ::Outcome<void, AZStd::string> StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal);

       virtual AZ::Outcome<void, AZStd::string> CancelTrajectoryGoal(TrajectoryResultPtr result);

       virtual ManipulatorActionStatus GetGoalStatus();
   };

   using ManipulatorTrajectoryRequestBus = AZ::EBus<ManipulatorTrajectoryRequests>;
} // namespace ROS2
