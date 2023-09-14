/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "InertialsMaker.h"
#include "RobotImporter/Utils/DefaultSolverConfiguration.h"
#include <AzCore/Component/EntityId.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <Source/EditorRigidBodyComponent.h>

namespace ROS2
{
    void InertialsMaker::AddInertial(const gz::math::Inertiald& inertial, AZ::EntityId entityId) const
    {
        AZ_Trace("AddInertial", "Processing inertial for entity id: %s\n", entityId.ToString().c_str());

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        PhysX::EditorRigidBodyConfiguration rigidBodyConfiguration;
        PhysX::RigidBodyConfiguration physxSpecificConfiguration;
        physxSpecificConfiguration.m_solverPositionIterations =
            AZStd::max(physxSpecificConfiguration.m_solverPositionIterations, URDF::DefaultNumberPosSolver);
        physxSpecificConfiguration.m_solverVelocityIterations =
            AZStd::max(physxSpecificConfiguration.m_solverVelocityIterations, URDF::DefaultNumberVelSolver);

        rigidBodyConfiguration.m_mass = inertial.MassMatrix().Mass();
        rigidBodyConfiguration.m_computeMass = false;

        rigidBodyConfiguration.m_centerOfMassOffset = URDF::TypeConversions::ConvertVector3(inertial.Pose().Pos());
        rigidBodyConfiguration.m_computeCenterOfMass = false;

        if (!URDF::TypeConversions::ConvertQuaternion(inertial.Pose().Rot()).IsIdentity())
        { // There is a rotation component in URDF that we are not able to apply
            AZ_Warning("AddInertial", false, "Ignoring URDF/SDF inertial origin rotation (no such field in rigid body configuration)");
        }

        auto moi = inertial.Moi();

        // Inertia tensor is symmetrical
        auto inertiaMatrix = AZ::Matrix3x3::CreateFromRows(
            AZ::Vector3(moi(0, 0), moi(0, 1), moi(0, 2)),
            AZ::Vector3(moi(0, 1), moi(1, 1), moi(1, 2)),
            AZ::Vector3(moi(0, 2), moi(1, 2), moi(2, 2)));
        rigidBodyConfiguration.m_inertiaTensor = inertiaMatrix;
        rigidBodyConfiguration.m_computeInertiaTensor = false;

        entity->CreateComponent<PhysX::EditorRigidBodyComponent>(rigidBodyConfiguration, physxSpecificConfiguration);
    }
} // namespace ROS2
