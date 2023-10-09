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
#include <AzCore/Outcome/Outcome.h>
#include <control_msgs/action/follow_joint_trajectory.hpp>

namespace ROS2
{
    //! Interface for commanding a system of joints such as robotic arm (manipulator) through FollowJointTrajectory actions.
    //@see <a href="https://github.com/ros-controls/control_msgs/blob/humble/control_msgs/action/FollowJointTrajectory.action">FollowJointTrajectory</a>
    class JointsTrajectoryRequests : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        using TrajectoryGoal = control_msgs::action::FollowJointTrajectory::Goal;
        using TrajectoryGoalPtr = std::shared_ptr<const TrajectoryGoal>;
        using TrajectoryResult = control_msgs::action::FollowJointTrajectory::Result;
        using TrajectoryResultPtr = std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result>;

        //! Status of trajectory action.
        enum class TrajectoryActionStatus
        {
            Idle, //!< No action has been ordered yet.
            Executing, //!< Ongoing trajectory goal.
            Cancelled, //!< Cancelled goal, either by the client user or simulation side.
            Succeeded //!< Goal reached.
        };

        //! Validate and, if validation is successful, start a trajectory goal.
        //! @param trajectoryGoal Specified trajectory including points with timing and tolerances.
        //! @return Nothing on success, error message if failed.
        //! @note The call will return an error if the goal trajectory mismatches joints system.
        virtual AZ::Outcome<void, TrajectoryResult> StartTrajectoryGoal(TrajectoryGoalPtr trajectoryGoal) = 0;

        //! Cancel current trajectory goal.
        //! @param result Result of trajectory goal with explanation on why it was cancelled.
        //! @return nothing on success, error if the goal could not be cancelled.
        virtual AZ::Outcome<void, AZStd::string> CancelTrajectoryGoal() = 0;

        //! Retrieve current trajectory goal status.
        //! @return Status of trajectory goal.
        virtual TrajectoryActionStatus GetGoalStatus() = 0;
    };

    using JointsTrajectoryRequestBus = AZ::EBus<JointsTrajectoryRequests>;
} // namespace ROS2
