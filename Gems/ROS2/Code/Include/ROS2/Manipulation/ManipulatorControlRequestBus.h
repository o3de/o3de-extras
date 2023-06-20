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
#include <AzCore/Name/Name.h>
#include <AzCore/std/utils.h>
#include <PhysX/ArticulationTypes.h>

namespace ROS2
{
   //! Interface for general requests for manipulator arms.
   //! This interface supports only manipulators with joints or articulation links with a single degree of freedom (DOF) each.
   class ManipulatorRequests : public AZ::EBusTraits
   {
   public:
       using BusIdType = AZ::EntityId;
       static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

       //! Structure that holds joint info for either articulation link or classical joint.
       struct JointInfo
       {
           bool m_isArticulation = false;
           PhysX::ArticulationJointAxis m_axis = PhysX::ArticulationJointAxis::Twist;
           AZ::EntityComponentIdPair m_entityComponentIdPair;
           float m_restPosition = 0.0f; //!< Keeps this position if no commands are given (for example, opposing gravity).
       };

       using ManipulatorJoints = AZ::unordered_map<AZ::Name, JointInfo>;

       //! Get manipulator joints, including joint or articulation component hierarchy.
       //! @return A map of joint names to joint info structure.
       virtual ManipulatorJoints GetManipulatorJoints();

       //! Get position of single degree of freedom joint by name.
       //! Works with hinge joints and articulation links.
       //! @param jointName name of the joint. Use names acquired from GetManipulatorJoints() query.
       //! @return outcome with relative position in degree of motion range if joint exists.
       //! If it does not exist or some other error happened, error message is returned.
       virtual AZ::Outcome<float, AZStd::string> GetSingleDOFJointPosition(const AZ::Name& jointName);

       //! Move a single degree of freedom joint into desired relative position.
       //! @param jointName name of the joint. Use names acquired from GetManipulatorJoints() query.
       //! @param position relative position in degree of motion range to achieve.
       //! @return nothing on success, error message on failure.
       //! @note the movement is realized by a specific controller and not instant. The manipulator will then keep this position.
       AZ::Outcome<void, AZStd::string> MoveSingleDOFJointToPosition(const AZ::Name& jointName, float position);

       //! Stop the manipulator. It will keep the position in which it stopped.
       void Stop();
   }
   using ManipulatorRequestBus = AZ::EBus<ManipulatorRequests>;
} // namespace ROS2
