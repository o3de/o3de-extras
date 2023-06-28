/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
#include "JointsManipulationComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Manipulation/JointInfo.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>

namespace ROS2
{
    namespace Internal
    {
        void AddHingeJointInfo(const AZ::EntityComponentIdPair& idPair, const AZStd::string& jointName, ManipulationJoints& joints)
        {
            if (joints.find(jointName) != joints.end())
            {
                AZ_Assert(false, "Joint names in hierarchy need to be unique (%s is not)!", jointName.c_str());
                return;
            }
            AZ_Printf("JointsManipulationEditorComponent", "Adding joint info for hinge joint %s\n", jointName.c_str());
            JointInfo jointInfo;
            jointInfo.m_isArticulation = false;
            jointInfo.m_axis = static_cast<PhysX::ArticulationJointAxis>(0);
            jointInfo.m_entityComponentIdPair = idPair;
            joints[jointName] = jointInfo;
        }

        bool TryGetFreeArticulationAxis(const AZ::EntityId& entityId, PhysX::ArticulationJointAxis& axis)
        {
            for (AZ::u8 i = 0; i <= static_cast<AZ::u8>(PhysX::ArticulationJointAxis::Z); i++)
            {
                PhysX::ArticulationJointMotionType type = PhysX::ArticulationJointMotionType::Locked;
                axis = static_cast<PhysX::ArticulationJointAxis>(i);
                // Use bus to prevent compilation error without PhysX Articulation support.
                PhysX::ArticulationJointRequestBus::EventResult(type, entityId, &PhysX::ArticulationJointRequests::GetMotion, axis);
                if (type != PhysX::ArticulationJointMotionType::Locked)
                {
                    return true;
                }
            }
            return false;
        }

        void AddArticulationJointInfo(const AZ::EntityComponentIdPair& idPair, const AZStd::string& jointName, ManipulationJoints& joints)
        {
            PhysX::ArticulationJointAxis freeAxis;
            bool hasFreeAxis = TryGetFreeArticulationAxis(idPair.GetEntityId(), freeAxis);
            if (!hasFreeAxis)
            { // Do not add a joint since it is a fixed one
                AZ_Printf("JointsManipulationEditorComponent", "Articulation joint %s is fixed, skipping\n", jointName.c_str());
                return;
            }

            if (joints.find(jointName) != joints.end())
            {
                AZ_Assert(false, "Joint names in hierarchy need to be unique (%s is not)!", jointName.c_str());
                return;
            }

            AZ_Printf("JointsManipulationEditorComponent", "Adding joint info for articulation link %s\n", jointName.c_str());
            JointInfo jointInfo;
            jointInfo.m_isArticulation = true;
            jointInfo.m_axis = freeAxis;
            jointInfo.m_entityComponentIdPair = idPair;
            joints[jointName] = jointInfo;
        }

        ManipulationJoints GetAllEntityHierarchyJoints(const AZ::EntityId& entityId)
        { // Look for either Articulation Links or Hinge joints in entity hierarchy and collect them into a map.
            // Determine kind of joints through presence of appropriate controller
            bool supportsArticulation = false;
            bool supportsClassicJoints = false;
            JointsPositionControllerRequestBus::EventResult(
                supportsArticulation, entityId, &JointsPositionControllerRequests::SupportsArticulation);
            JointsPositionControllerRequestBus::EventResult(
                supportsClassicJoints, entityId, &JointsPositionControllerRequests::SupportsClassicJoints);
            ManipulationJoints manipulationJoints;
            if (!supportsArticulation && !supportsClassicJoints)
            {
                AZ_Warning("JointsManipulationEditorComponent", false, "No suitable Position Controller Component in entity!");
                return manipulationJoints;
            }
            if (supportsArticulation && supportsClassicJoints)
            {
                AZ_Warning(
                    "JointsManipulationEditorComponent", false, "Cannot support both classic joint and articulations in one hierarchy");
                return manipulationJoints;
            }

            // Get all descendants and iterate over joints
            AZStd::vector<AZ::EntityId> descendants;
            AZ::TransformBus::EventResult(descendants, entityId, &AZ::TransformInterface::GetEntityAndAllDescendants);
            AZ_Warning(
                "JointsManipulationEditorComponent", descendants.size() > 0, "Entity %s has no descendants!", entityId.ToString().c_str());
            for (const AZ::EntityId& descendantID : descendants)
            {
                AZ::Entity* entity = nullptr;
                AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, descendantID);
                AZ_Assert(entity, "Unknown entity %s", descendantID.ToString().c_str());

                // If there is a Frame Component, take joint name stored in it.
                auto* frameComponent = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
                if (!frameComponent)
                { // Frame Component is required for joints.
                    continue;
                }
                const AZStd::string jointName(frameComponent->GetJointName().GetCStr());

                auto* hingeComponent = Utils::GetGameOrEditorComponent<PhysX::HingeJointComponent>(entity);
                auto* articulationComponent = Utils::GetGameOrEditorComponent<PhysX::ArticulationLinkComponent>(entity);
                AZ_Warning(
                    "JointsManipulationEditorComponent",
                    hingeComponent && !supportsClassicJoints,
                    "Found classic joints but the controller does not support them!");
                AZ_Warning(
                    "JointsManipulationEditorComponent",
                    articulationComponent && !supportsArticulation,
                    "Found articulations but the controller does not support them!");

                // See if there is a Hinge Joint in the entity, add it to map.
                if (supportsClassicJoints && hingeComponent)
                {
                    auto idPair = AZ::EntityComponentIdPair(hingeComponent->GetEntityId(), hingeComponent->GetId());
                    Internal::AddHingeJointInfo(idPair, jointName, manipulationJoints);
                }

                // See if there is an Articulation Link in the entity, add it to map.
                if (supportsArticulation && articulationComponent)
                {
                    auto idPair = AZ::EntityComponentIdPair(articulationComponent->GetEntityId(), articulationComponent->GetId());
                    Internal::AddArticulationJointInfo(idPair, jointName, manipulationJoints);
                }
            }
            return manipulationJoints;
        }

        void SetInitialPositions(ManipulationJoints& manipulationJoints, const AZStd::unordered_map<AZStd::string, float>& initialPositions)
        {
            // Set the initial / resting position to move to and keep.
            for (const auto& [jointName, jointInfo] : manipulationJoints)
            {
                AZ_Warning(
                    "JointsManipulationEditorComponent",
                    initialPositions.find(jointName.c_str()) != initialPositions.end(),
                    "No set initial position for joint %s",
                    jointName.c_str());
                manipulationJoints[jointName].m_restPosition = initialPositions.at(jointName);
            }
        }
    } // namespace Internal

    JointsManipulationEditorComponent::JointsManipulationEditorComponent()
    {
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
        m_jointStatePublisherConfiguration.m_frequency = 25.0f;
    }

    void JointsManipulationEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        auto manipulationJoints = Internal::GetAllEntityHierarchyJoints(GetEntityId());
        Internal::SetInitialPositions(manipulationJoints, m_initialPositions);
        gameEntity->CreateComponent<JointsManipulationComponent>(m_jointStatePublisherConfiguration, manipulationJoints);
    }

    void JointsManipulationEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsManipulationEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationEditorComponent, AZ::Component>()
                ->Version(0)
                ->Field("JointStatePublisherConfiguration", &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration)
                ->Field("Initial positions", &JointsManipulationEditorComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsManipulationEditorComponent>("JointsManipulationEditorComponent", "Component for manipulation of joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration,
                        "Joint State Publisher",
                        "Configuration of Joint State Publisher")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_initialPositions,
                        "Initial positions",
                        "Initial positions of all the joints");
            }
        }
    }
} // namespace ROS2
