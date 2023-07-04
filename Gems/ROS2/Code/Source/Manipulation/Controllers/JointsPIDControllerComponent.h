/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>

namespace ROS2
{
    //! Handles position control commands for joints.
    class JointsPIDControllerComponent
        : public AZ::Component
        , public JointsPositionControllerRequestBus::Handler
    {
    public:
        JointsPIDControllerComponent() = default;
        ~JointsPIDControllerComponent() = default;
        AZ_COMPONENT(JointsPIDControllerComponent, "{41A31EDB-90B0-412E-BBFA-D35D45546A8E}", AZ::Component);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // JointsPositionControllerRequestBus::Handler overrides ...
        //! @see ROS2::JointsPositionControllerRequestBus::SupportsArticulation
        bool SupportsArticulation() override
        {
            return false;
        }

        //! @see ROS2::JointsPositionControllerRequestBus::SupportsClassicJoints
        bool SupportsClassicJoints() override
        {
            return true;
        }

        //! @see ROS2::JointsPositionControllerRequestBus::PositionControl
        AZ::Outcome<void, AZStd::string> PositionControl(
            const AZStd::string& jointName,
            JointInfo joint,
            JointPosition currentPosition,
            JointPosition targetPosition,
            float deltaTime) override;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;
        void InitializePIDs();

        AZStd::unordered_map<AZStd::string, Controllers::PidConfiguration> m_pidConfiguration;
    };
} // namespace ROS2
