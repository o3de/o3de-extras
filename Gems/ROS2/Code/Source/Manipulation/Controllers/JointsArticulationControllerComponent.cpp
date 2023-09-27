/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsArticulationControllerComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <PhysX/ArticulationJointBus.h>

namespace ROS2
{
    void JointsArticulationControllerComponent::Activate()
    {
        JointsPositionControllerRequestBus::Handler::BusConnect(GetEntityId());
    }

    void JointsArticulationControllerComponent::Deactivate()
    {
        JointsPositionControllerRequestBus::Handler::BusDisconnect();
    }

    AZ::Outcome<void, AZStd::string> JointsArticulationControllerComponent::PositionControl(
        const AZStd::string& jointName,
        JointInfo joint,
        [[maybe_unused]] JointPosition currentPosition,
        JointPosition targetPosition,
        [[maybe_unused]] float deltaTime)
    {
        if (!joint.m_isArticulation)
        {
            return AZ::Failure(AZStd::string::format(
                "Joint %s is not an articulation link, use JointsPIDControllerComponent instead", jointName.c_str()));
        }

        PhysX::ArticulationJointRequestBus::Event(
            joint.m_entityComponentIdPair.GetEntityId(), &PhysX::ArticulationJointRequests::SetDriveTarget, joint.m_axis, targetPosition);
        return AZ::Success();
    }

    void JointsArticulationControllerComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ArticulationLinkService"));
    }

    void JointsArticulationControllerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsArticulationControllerComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsArticulationControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsArticulationControllerComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsArticulationControllerComponent>(
                      "JointsArticulationControllerComponent", "Component to move articulation joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/JointsArticulationControllerComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/JointsArticulationControllerComponent.svg");
            }
        }
    }
} // namespace ROS2
