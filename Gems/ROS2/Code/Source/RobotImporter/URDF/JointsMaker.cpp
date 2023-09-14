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
        const sdf::Joint* joint, AZ::EntityId followColliderEntityId, AZ::EntityId leadColliderEntityId) const
    {
        AZ::Entity* followColliderEntity = AzToolsFramework::GetEntityById(followColliderEntityId);
        PhysX::EditorJointComponent* jointComponent = nullptr;

        // URDF/SDF has a joint axis configurable by a normalized vector - that is given by the 'axis' sub-element in the joint element.
        // The o3de has a slightly different way of configuring the axis of the joint. The o3de has an axis around positive `X` and rotation
        // with Euler angles can be applied to configure the desirable direction of the joint. A quaternion that transforms a unit vector X
        // {1,0,0} to a vector given by the URDF/SDF joint need to be found. Heavily suboptimal element in this conversion is needed of
        // converting the unit quaternion to Euler vector.
        const AZ::Vector3 o3deJointDir{ 1.0, 0.0, 0.0 };
        const sdf::JointAxis* jointAxis = joint->Axis();
        AZ::Vector3 jointCoordinateAxis = AZ::Vector3::CreateZero();
        auto quaternion = AZ::Quaternion::CreateIdentity();
        if (jointAxis != nullptr)
        {
            jointCoordinateAxis = URDF::TypeConversions::ConvertVector3(jointAxis->Xyz());
            quaternion =
                jointCoordinateAxis.IsZero() ? AZ::Quaternion::CreateIdentity() : AZ::Quaternion::CreateShortestArc(o3deJointDir, jointCoordinateAxis);
        }

        AZ_Printf(
            "JointsMaker",
            "Quaternion from URDF/SDF to o3de %f, %f, %f, %f",
            quaternion.GetX(),
            quaternion.GetY(),
            quaternion.GetZ(),
            quaternion.GetW());
        const AZ::Vector3 rotation = quaternion.GetEulerDegrees();

        switch (joint->Type())
        {
        case sdf::JointType::FIXED:
            {
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorFixedJointComponent>();
            }
            break;
        case sdf::JointType::PRISMATIC:
            {
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorPrismaticJointComponent>();
                followColliderEntity->Activate();

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetVector3Value,
                    PhysX::JointsComponentModeCommon::ParameterNames::Rotation,
                    rotation);

                if (jointAxis != nullptr)
                {
                    PhysX::EditorJointRequestBus::Event(
                        AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                        &PhysX::EditorJointRequests::SetLinearValuePair,
                        PhysX::JointsComponentModeCommon::ParameterNames::LinearLimits,
                        PhysX::AngleLimitsFloatPair(jointAxis->Upper(), jointAxis->Lower()));
                }
                followColliderEntity->Deactivate();
            }
            break;
        case sdf::JointType::CONTINUOUS:
            { // Implemented as Hinge with angular limit disabled
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                followColliderEntity->Activate();

                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                    [&rotation](PhysX::EditorJointRequests* editorJointRequest)
                    {
                        editorJointRequest->SetVector3Value(PhysX::JointsComponentModeCommon::ParameterNames::Rotation, rotation);
                        editorJointRequest->SetLinearValuePair(
                            PhysX::JointsComponentModeCommon::ParameterNames::TwistLimits,
                            PhysX::AngleLimitsFloatPair(AZ::RadToDeg(AZ::Constants::TwoPi), -AZ::RadToDeg(AZ::Constants::TwoPi)));
                        editorJointRequest->SetBoolValue(PhysX::JointsComponentModeCommon::ParameterNames::EnableLimits, false);
                    });
                followColliderEntity->Deactivate();
            }
            break;
        case sdf::JointType::REVOLUTE:
            { // Hinge
                jointComponent = followColliderEntity->CreateComponent<PhysX::EditorHingeJointComponent>();
                followColliderEntity->Activate();

                if (jointAxis != nullptr)
                {
                    using LimitType = decltype(jointAxis->Upper());
                    const bool enableJointLimits = jointAxis->Upper() != AZStd::numeric_limits<LimitType>::infinity()
                        || jointAxis->Lower() != -AZStd::numeric_limits<LimitType>::infinity();
                    const double limitUpper = jointAxis->Upper() != AZStd::numeric_limits<LimitType>::infinity()
                        ? AZ::RadToDeg(jointAxis->Upper())
                        : AZ::RadToDeg(AZ::Constants::TwoPi);
                    const double limitLower = jointAxis->Lower() != -AZStd::numeric_limits<LimitType>::infinity()
                        ? AZ::RadToDeg(jointAxis->Lower())
                        : -AZ::RadToDeg(AZ::Constants::TwoPi);
                    AZ_Printf(
                        "JointsMaker",
                        "Setting limits : upper: %.1f lower: %.1f (URDF/SDF:%f,%f)",
                        limitUpper,
                        limitLower,
                        jointAxis->Upper(),
                        jointAxis->Lower());
                    PhysX::EditorJointRequestBus::Event(
                        AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
                        [&rotation, enableJointLimits, &limitLower, &limitUpper](PhysX::EditorJointRequests* editorJointRequest)
                        {
                            editorJointRequest->SetVector3Value(PhysX::JointsComponentModeCommon::ParameterNames::Rotation, rotation);
                            editorJointRequest->SetLinearValuePair(
                                PhysX::JointsComponentModeCommon::ParameterNames::TwistLimits,
                                PhysX::AngleLimitsFloatPair(limitUpper, limitLower));
                            editorJointRequest->SetBoolValue(
                                PhysX::JointsComponentModeCommon::ParameterNames::EnableLimits, enableJointLimits);
                        });
                }
                followColliderEntity->Deactivate();
            }
            break;
        default:
            AZ_Warning("AddJointComponent", false, "Unknown or unsupported joint type %d for joint %s",
                static_cast<int>(joint->Type()), joint->Name().c_str());
            return AZ::Failure(AZStd::string::format("unsupported joint type : %d", static_cast<int>(joint->Type())));
        }

        followColliderEntity->Activate();
        PhysX::EditorJointRequestBus::Event(
            AZ::EntityComponentIdPair(followColliderEntityId, jointComponent->GetId()),
            &PhysX::EditorJointRequests::SetEntityIdValue,
            PhysX::JointsComponentModeCommon::ParameterNames::LeadEntity,
            leadColliderEntityId);
        followColliderEntity->Deactivate();
        return AZ::Success(jointComponent->GetId());
    }
} // namespace ROS2
