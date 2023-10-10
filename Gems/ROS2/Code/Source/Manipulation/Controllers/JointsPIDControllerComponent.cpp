/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsPIDControllerComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>

namespace ROS2
{
    void JointsPIDControllerComponent::Activate()
    {
        JointsPositionControllerRequestBus::Handler::BusConnect(GetEntityId());
        InitializePIDs();
    }

    void JointsPIDControllerComponent::Deactivate()
    {
        JointsPositionControllerRequestBus::Handler::BusDisconnect();
    }

    void JointsPIDControllerComponent::InitializePIDs()
    {
        for (auto& [jointName, pid] : m_pidConfiguration)
        {
            pid.InitializePid();
        }
    }

    AZ::Outcome<void, AZStd::string> JointsPIDControllerComponent::PositionControl(
        const AZStd::string& jointName,
        JointInfo joint,
        JointPosition currentPosition,
        JointPosition targetPosition,
        float deltaTime)
    {
        if (joint.m_isArticulation)
        {
            return AZ::Failure(AZStd::string::format("Joint %s is articulation link, JointsPIDControllerComponent only handles classic Hinge joints. Use "
                               "JointsArticulationControllerComponent instead", jointName.c_str()));
        }

        bool jointPIDdefined = m_pidConfiguration.find(jointName) != m_pidConfiguration.end();
        AZ_Warning(
            "JointsPIDControllerComponent",
            jointPIDdefined,
            "PID not defined for joint %s, using a default, the behavior is likely to be wrong for this joint",
            jointName.c_str());

        Controllers::PidConfiguration defaultConfiguration;
        defaultConfiguration.InitializePid();
        auto applicablePidConfiguration = jointPIDdefined ? m_pidConfiguration.at(jointName) : defaultConfiguration;

        uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        auto positionError = targetPosition - currentPosition;
        float desiredVelocity = applicablePidConfiguration.ComputeCommand(positionError, deltaTimeNs);
        PhysX::JointRequestBus::Event(joint.m_entityComponentIdPair, &PhysX::JointRequests::SetVelocity, desiredVelocity);
        return AZ::Success();
    }

    void JointsPIDControllerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsPIDControllerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsPIDControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsPIDControllerComponent, AZ::Component>()->Version(0)->Field(
                "JointsPIDs", &JointsPIDControllerComponent::m_pidConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsPIDControllerComponent>("JointsPIDControllerComponent", "Component to move joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/JointsPIDControllerComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/JointsPIDControllerComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsPIDControllerComponent::m_pidConfiguration,
                        "Joint PIDs",
                        "PID configuration for each free joint in this entity hierarchy");
            }
        }
    }
} // namespace ROS2
