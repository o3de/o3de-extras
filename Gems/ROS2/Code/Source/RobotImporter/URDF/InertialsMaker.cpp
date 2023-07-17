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
    void InertialsMaker::AddInertial(urdf::InertialSharedPtr inertial, AZ::EntityId entityId) const
    {
        if (!inertial)
        { // it is ok not to have inertia in a link
            return;
        }
        AZ_Trace("AddInertial", "Processing inertial for entity id: %s\n", entityId.ToString().c_str());

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        PhysX::EditorRigidBodyConfiguration rigidBodyConfiguration;
        PhysX::RigidBodyConfiguration physxSpecificConfiguration;
        physxSpecificConfiguration.m_solverPositionIterations =
            AZStd::max(physxSpecificConfiguration.m_solverPositionIterations, URDF::DefaultNumberPosSolver);
        physxSpecificConfiguration.m_solverVelocityIterations =
            AZStd::max(physxSpecificConfiguration.m_solverVelocityIterations, URDF::DefaultNumberVelSolver);

        rigidBodyConfiguration.m_mass = inertial->mass;
        rigidBodyConfiguration.m_computeMass = false;

        rigidBodyConfiguration.m_centerOfMassOffset = URDF::TypeConversions::ConvertVector3(inertial->origin.position);
        rigidBodyConfiguration.m_computeCenterOfMass = false;

        if (!URDF::TypeConversions::ConvertQuaternion(inertial->origin.rotation).IsIdentity())
        { // There is a rotation component in URDF that we are not able to apply
            AZ_Warning("AddInertial", false, "Ignoring URDF inertial origin rotation (no such field in rigid body configuration)");
        }

        // Inertia tensor is symmetrical
        auto inertiaMatrix = AZ::Matrix3x3::CreateFromRows(
            AZ::Vector3(inertial->ixx, inertial->ixy, inertial->ixz),
            AZ::Vector3(inertial->ixy, inertial->iyy, inertial->iyz),
            AZ::Vector3(inertial->ixz, inertial->iyz, inertial->izz));
        rigidBodyConfiguration.m_inertiaTensor = inertiaMatrix;
        rigidBodyConfiguration.m_computeInertiaTensor = false;

        entity->CreateComponent<PhysX::EditorRigidBodyComponent>(rigidBodyConfiguration, physxSpecificConfiguration);
    }
} // namespace ROS2
