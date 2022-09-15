/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/InertialsMaker.h"
#include "RobotImporter/URDF/TypeConversions.h"
#include <AzCore/Component/EntityId.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <Source/EditorRigidBodyComponent.h>

namespace ROS2
{
    void InertialsMaker::AddInertial(urdf::InertialSharedPtr inertial, AZ::EntityId entityId)
    {
        if (!inertial)
        { // it is ok not to have inertia in a link
            return;
        }
        AZ_TracePrintf("AddInertial", "Processing inertial for entity id:%s\n", entityId.ToString().c_str());

        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        PhysX::EditorRigidBodyConfiguration rigidBodyConfiguration;
        rigidBodyConfiguration.m_mass = inertial->mass;
        rigidBodyConfiguration.m_computeMass = false;

        // TODO - Check whether this is a correct offset for every case (consider entity transform and collider origin)
        rigidBodyConfiguration.m_centerOfMassOffset = URDF::TypeConversions::ConvertVector3(inertial->origin.position);
        rigidBodyConfiguration.m_computeCenterOfMass = false;

        if (!URDF::TypeConversions::ConvertQuaternion(inertial->origin.rotation).IsIdentity())
        { // There is a rotation component in URDF that we are not able to apply
            // TODO - determine a solution to also include rotation in import
            AZ_Warning("AddInertial", false, "Ignoring URDF inertial origin rotation (no such field in rigid body configuration)");
        }

        // Inertia tensor is symmetrical
        auto inertiaMatrix = AZ::Matrix3x3::CreateFromRows(
            AZ::Vector3(inertial->ixx, inertial->ixy, inertial->ixz),
            AZ::Vector3(inertial->ixy, inertial->iyy, inertial->iyz),
            AZ::Vector3(inertial->ixz, inertial->iyz, inertial->izz));
        rigidBodyConfiguration.m_inertiaTensor = inertiaMatrix;
        rigidBodyConfiguration.m_computeInertiaTensor = false;

        entity->CreateComponent<PhysX::EditorRigidBodyComponent>(rigidBodyConfiguration);
    }
} // namespace ROS2
