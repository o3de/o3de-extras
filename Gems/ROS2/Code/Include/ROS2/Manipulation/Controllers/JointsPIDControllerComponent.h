/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Name/Name.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>

namespace ROS2
{
   //! Handles position control commands for joints.
   class JointPIDControllerComponent
       : public AZ::Component
       , public JointsPositionControllerRequestBus::Handler
   {
   public:
       JointPIDControllerComponent() = default;
       ~JointPIDControllerComponent() = default;
       AZ_COMPONENT(JointPIDControllerComponent, "{41A31EDB-90B0-412E-BBFA-D35D45546A8E}", AZ::Component);

       static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
       static void Reflect(AZ::ReflectContext* context);

       // JointsPositionControllerRequestBus::Handler overrides ...
       //! @see ROS2::JointsPositionControllerRequestBus::PositionControl
       AZ::Outcome<void, AZStd::string> PositionControl(
           JointManipulationRequests::JointInfo joint,
           JointManipulationRequests::JointPosition currentPosition,
           JointManipulationRequests::JointPosition targetPosition,
           float deltaTime) override;

   private:
       // Component overrides ...
       void Activate() override;
       void Deactivate() override;
       void InitializePIDs();

       // TODO - temporary solution. Use Editor Component and check against named joints. (populate names / update initially, watch bus).
       AZStd::unordered_map<AZ::Name, Controllers::PidConfiguration> m_pidConfigurationVector;
   };
} // namespace ROS2
