/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsMaker.h"
#include "PrefabMakerUtils.h"
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorFixedJointComponent.h>
#include <Source/EditorHingeJointComponent.h>
#include <Source/EditorPrismaticJointComponent.h>
#include <Source/EditorRigidBodyComponent.h>

namespace ROS2
{

    JointsMaker::JointsMakerResult JointsMaker::AddJointComponent(
        urdf::JointSharedPtr joint, AZ::EntityId followColliderEntityId, AZ::EntityId leadColliderEntityId) const
    {
        AZ::Entity* followColliderEntity = AzToolsFramework::GetEntityById(followColliderEntityId);
        PhysX::EditorJointComponent* jointComponent = nullptr;

        // URDF has a joint axis configurable by a normalized vector - that is given by the 'axis' sub-element in the joint element.
        // The o3de has a slightly different way of configuring the axis of the joint. The o3de has an axis around positive `X` and rotation
        // with Euler angles can be applied to configure the desirable direction of the joint. A quaternion that transforms a unit vector X
        // {1,0,0} to a vector given by the URDF joint need to be found. Heavily suboptimal element in this conversion is needed of
        // converting the unit quaternion to Euler vector.
        const AZ::Vector3 o3de_joint_dir{ 1.0, 0.0, 0.0 };
        const AZ::Vector3 joint_axis = URDF::TypeConversions::ConvertVector3(joint->axis);
        const auto quaternion =
            joint_axis.IsZero() ? AZ::Quaternion::CreateIdentity() : AZ::Quaternion::CreateShortestArc(o3de_joint_dir, joint_axis);

        AZ_Printf(
            "JointsMaker",
            "Quaternion from URDF to o3de %f, %f, %f, %f",
            quaternion.GetX(),
            quaternion.GetY(),
            quaternion.GetZ(),
            quaternion.GetW());
        const AZ::Vector3 rotation = quaternion.GetEulerDegrees();

        switch (joint->type)
        {
        case urdf::Joint::FIXED:
            {
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorFixedJointComponent>();
            }
            break;
        case urdf::Joint::PRISMATIC:
            {
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorPrismaticJointComponent>();
                followColliderEntity->Activate();

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetVector3Value,
                    PhysX::JointsComponentModeCommon::ParamaterNames::Rotation,
                    rotation);

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetLinearValuePair,
                    PhysX::JointsComponentModeCommon::ParamaterNames::LinearLimits,
                    PhysX::AngleLimitsFloatPair(joint->limits->upper, joint->limits->lower));

                followColliderEntity->Deactivate();
            }
            break;
        case urdf::Joint::CONTINUOUS:
            { // Implemented as Hinge with angular limit disabled
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                followColliderEntity->Activate();

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetVector3Value,
                    PhysX::JointsComponentModeCommon::ParamaterNames::Rotation,
                    rotation);

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetLinearValuePair,
                    PhysX::JointsComponentModeCommon::ParamaterNames::TwistLimits,
                    PhysX::AngleLimitsFloatPair(AZ::RadToDeg(AZ::Constants::TwoPi), -AZ::RadToDeg(AZ::Constants::TwoPi)));

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetBoolValue,
                    PhysX::JointsComponentModeCommon::ParamaterNames::EnableLimits,
                    false);

                followColliderEntity->Deactivate();
            }
            break;
        case urdf::Joint::REVOLUTE:
            { // Hinge
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                followColliderEntity->Activate();

                const double limitUpper = AZ::RadToDeg(joint->limits->upper);
                const double limitLower = AZ::RadToDeg(joint->limits->lower);
                AZ_Printf(
                    "JointsMaker",
                    "Setting limits : upper: %.1f lower: %.1f (URDF:%f,%f)",
                    limitUpper,
                    limitLower,
                    joint->limits->upper,
                    joint->limits->lower);
                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetVector3Value,
                    PhysX::JointsComponentModeCommon::ParamaterNames::Rotation,
                    rotation);

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetLinearValuePair,
                    PhysX::JointsComponentModeCommon::ParamaterNames::TwistLimits,
                    PhysX::AngleLimitsFloatPair(limitUpper, limitLower));

                followColliderEntity->Deactivate();
            }
            break;
        default:
            AZ_Warning("AddJointComponent", false, "Unknown or unsupported joint type %d for joint %s", joint->type, joint->name.c_str());
            return AZ::Failure(AZStd::string::format("unsupported joint type : %d", joint->type));
        }

        followColliderEntity->Activate();
        PhysX::EditorJointRequestBus::Event(
            AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
            &PhysX::EditorJointRequests::SetEntityIdValue,
            PhysX::JointsComponentModeCommon::ParamaterNames::LeadEntity,
            leadColliderEntityId);
        followColliderEntity->Deactivate();
        return AZ::Success(jointComponent->GetId());
    }
} // namespace ROS2
