/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/JointsMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <Source/EditorFixedJointComponent.h>
#include <Source/EditorHingeJointComponent.h>
#include <Source/EditorShapeColliderComponent.h>

namespace ROS2
{
    void JointsMaker::AddJoint(
        urdf::LinkSharedPtr parentLink, urdf::LinkSharedPtr childLink, AZ::EntityId linkChildId, AZ::EntityId linkParentId)
    { // Find if there is a joint between this child and its parent, add / change relevant components
        for (auto joint : parentLink->child_joints)
        {
            if (joint->child_link_name == childLink->name)
            { // Found a match!
                PrefabMakerUtils::SetEntityTransform(joint->parent_to_joint_origin_transform, linkChildId);
                return AddJoint(joint, linkChildId, linkParentId);
            }
        }
    }

    void JointsMaker::AddJoint(urdf::JointSharedPtr joint, AZ::EntityId linkChildId, AZ::EntityId linkParentId)
    {
        if (!PrefabMakerUtils::HasCollider(linkChildId) || !PrefabMakerUtils::HasCollider(linkParentId))
        { // This check should always pass unless adding colliders earlier failed for some reason
            AZ_Error("AddJoint", false, "Unable to add a joint %s without lead and follow colliders", joint->name.c_str());
            return;
        }
        AddJointComponent(joint, linkChildId, linkParentId);
    }

    void JointsMaker::AddJointComponent(urdf::JointSharedPtr joint, AZ::EntityId followColliderEntityId, AZ::EntityId leadColliderEntityId)
    {
        AZ::Entity* followColliderEntity = AzToolsFramework::GetEntityById(followColliderEntityId);
        PhysX::EditorJointComponent* jointComponent = nullptr;
        // TODO - ATM, there is no support withing Joint Components for the following:
        // TODO <calibration> <dynamics> <mimic>, friction, effort, velocity, joint safety and several joint types
        // TODO - apply <axis>
        switch (joint->type)
        { // TODO - replace with a generic member function
        case urdf::Joint::FIXED:
            {
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorFixedJointComponent>();
            }
            break;
        case urdf::Joint::CONTINUOUS:
            { // Implemented as Hinge with limit set to -2*PI and 2*PI deg. Works fine in the Engine
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                followColliderEntity->Activate();
                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetLinearValuePair,
                    PhysX::JointsComponentModeCommon::ParamaterNames::TwistLimits,
                    PhysX::AngleLimitsFloatPair(AZ::RadToDeg(AZ::Constants::TwoPi), -AZ::RadToDeg(AZ::Constants::TwoPi)));
                followColliderEntity->Deactivate();
            }
            break;
        case urdf::Joint::REVOLUTE:
            { // Hinge
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                followColliderEntity->Activate();
                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetLinearValuePair,
                    PhysX::JointsComponentModeCommon::ParamaterNames::TwistLimits,
                    PhysX::AngleLimitsFloatPair(AZ::RadToDeg(joint->limits->upper), AZ::RadToDeg(joint->limits->lower)));
                followColliderEntity->Deactivate();
            }
            break;
        default:
            AZ_Warning("AddJointComponent", false, "Unknown or unsupported joint type %d for joint %s", joint->type, joint->name.c_str());
            break;
        }

        if (!jointComponent)
        {
            return;
        }

        followColliderEntity->Activate();
        PhysX::EditorJointRequestBus::Event(
            AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
            &PhysX::EditorJointRequests::SetEntityIdValue,
            PhysX::JointsComponentModeCommon::ParamaterNames::LeadEntity,
            leadColliderEntityId);
        followColliderEntity->Deactivate();
    }
} // namespace ROS2
