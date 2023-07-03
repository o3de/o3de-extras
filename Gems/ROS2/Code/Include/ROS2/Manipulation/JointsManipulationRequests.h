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
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/utils.h>
#include <ROS2/Manipulation/JointInfo.h>

namespace ROS2
{
    //! Interface for general requests for joint systems such as manipulator arms.
    //! This interface supports only systems with joints or articulation links with a single degree of freedom (DOF) each.
    class JointsManipulationRequests : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Get all entity tree joints, including joint or articulation component hierarchy.
        //! @return A map of joint names to joint info structure.
        //! @note Only free joints are returned (no fixed ones).
        virtual ManipulationJoints GetJoints() = 0;

        //! Get position of a joint by name.
        //! Works with hinge joints and articulation links.
        //! @param jointName name of the joint. Use names acquired from GetJoints() query.
        //! @return outcome with relative position in degree of motion range if joint exists.
        //! If it does not exist or some other error happened, error message is returned.
        virtual AZ::Outcome<JointPosition, AZStd::string> GetJointPosition(const AZStd::string& jointName) = 0;

        //! Return positions of all single DOF joints.
        //! @return a vector of all joints relative positions in degree of motion range or error message.
        virtual AZStd::vector<JointPosition> GetAllJointsPositions() = 0;

        //! Move specified joints into positions.
        //! @param new positions for each named joint. Use names queried through GetJoints().
        //! @return nothing on success, error message on failure.
        //! @note the movement is realized by a specific controller and not instant. The joints will then keep these positions.
        virtual AZ::Outcome<void, AZStd::string> MoveJointsToPositions(
            const AZStd::unordered_map<AZStd::string, JointPosition> positions) = 0;

        //! Move a single joint into desired relative position.
        //! @param jointName name of the joint. Use names acquired from GetJoints() query.
        //! @param position relative position in degree of motion range to achieve.
        //! @return nothing on success, error message on failure.
        //! @note the movement is realized by a specific controller and not instant. The joints will then keep this position.
        virtual AZ::Outcome<void, AZStd::string> MoveJointToPosition(const AZStd::string& jointName, JointPosition position) = 0;

        //! Stop the joints movement in progress. It will keep the position in which it stopped.
        virtual void Stop() = 0;
    };
    using JointsManipulationRequestBus = AZ::EBus<JointsManipulationRequests>;
} // namespace ROS2
