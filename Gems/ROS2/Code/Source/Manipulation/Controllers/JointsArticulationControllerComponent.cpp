/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsArticulationControllerComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>

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
        [[maybe_unused]] const AZ::Name& jointName,
        JointManipulationRequests::JointInfo joint,
        [[maybe_unused]] JointManipulationRequests::JointPosition currentPosition,
        JointManipulationRequests::JointPosition targetPosition,
        [[maybe_unused]] float deltaTime)
    {
        if (!joint.m_isArticulation)
        { // TODO - this situation should be resolved through RequiredServices instead or otherwise through validation.
            return AZ::Failure("Joint %s is not an articulation link, use JointsPIDControllerComponent instead");
        }

        PhysX::ArticulationJointRequestBus::Event(
            joint.m_componentIdPair.GetEntityId(), &PhysX::ArticulationJointRequests::SetDriveTarget, joint.m_axis, targetPosition);
        return AZ::Success();
    }

    void JointsPIDControllerComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsPIDControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsPIDControllerComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsPIDControllerComponent>("JointsPIDControllerComponent", "Component to move joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }
} // namespace ROS2
