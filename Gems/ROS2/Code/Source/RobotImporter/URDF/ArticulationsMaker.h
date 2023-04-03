/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include "UrdfParser.h"
#include <AzCore/Component/EntityId.h>
#include <PhysX/ArticulationTypes.h>
#include <AzCore/std/containers/unordered_map.h>
namespace ROS2
{
   //! Populates the entity with contents of the <inertial> tag in robot description.
   class ArticulationsMaker
   {
   public:
       //! Add zero or one inertial elements to a given entity (depending on link content).
       //! @param link A pointer to a parsed URDF link
       //! @param entityId A non-active entity which will be populated according to inertial content.
       void AddArticulationLink(urdf::LinkSharedPtr link, AZ::EntityId entityId) const;
   private:
       const AZStd::unordered_map<int, PhysX::ArticulationJointType> SupportedJointTypes {{
           {urdf::Joint::REVOLUTE, PhysX::ArticulationJointType::Hinge},
           {urdf::Joint::CONTINUOUS, PhysX::ArticulationJointType::Hinge},
           {urdf::Joint::PRISMATIC, PhysX::ArticulationJointType::Prismatic},
           {urdf::Joint::FIXED, PhysX::ArticulationJointType::Fix},
       }};
   };
} // namespace ROS2
