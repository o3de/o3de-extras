/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ArticulationsMaker.h"
#include "RobotImporter/Utils/DefaultSolverConfiguration.h"
#include <AzCore/Component/EntityId.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <Source/EditorArticulationLinkComponent.h>

namespace ROS2
{

    namespace
    {
        using ArticulationCfg = PhysX::EditorArticulationLinkConfiguration;
        static const AZStd::unordered_map<int, PhysX::ArticulationJointType> SupportedJointTypes{ {
            { urdf::Joint::REVOLUTE, PhysX::ArticulationJointType::Hinge },
            { urdf::Joint::CONTINUOUS, PhysX::ArticulationJointType::Hinge },
            { urdf::Joint::PRISMATIC, PhysX::ArticulationJointType::Prismatic },
            { urdf::Joint::FIXED, PhysX::ArticulationJointType::Fix },
        } };
    } // namespace

    ArticulationCfg& AddToArticulationConfig(ArticulationCfg& articulationLinkConfiguration, const urdf::JointSharedPtr joint)
    {
        if (!joint)
        {
            return articulationLinkConfiguration;
        }
        auto supportedArticulationType = SupportedJointTypes.find(joint->type);
        AZ_Warning(
            "ArticulationsMaker",
            supportedArticulationType != SupportedJointTypes.end(),
            "Articulations do not support type %d for URDF joint %s.",
            joint->type,
            joint->name.c_str());
        if (supportedArticulationType != SupportedJointTypes.end())
        {
            const auto type = supportedArticulationType->second;
            articulationLinkConfiguration.m_articulationJointType = type;
            const AZ::Vector3 o3deJointDir{ 1.0, 0.0, 0.0 };
            const AZ::Vector3 jointAxis = URDF::TypeConversions::ConvertVector3(joint->axis);
            const auto quaternion =
                jointAxis.IsZero() ? AZ::Quaternion::CreateIdentity() : AZ::Quaternion::CreateShortestArc(o3deJointDir, jointAxis);
            const AZ::Vector3 rotation = quaternion.GetEulerDegrees();
            articulationLinkConfiguration.m_localRotation = rotation;

            if (joint->limits)
            {
                if (type == PhysX::ArticulationJointType::Hinge)
                {
                    const double limitUpper = AZ::RadToDeg(joint->limits->upper);
                    const double limitLower = AZ::RadToDeg(joint->limits->lower);
                    articulationLinkConfiguration.m_angularLimitNegative = limitLower;
                    articulationLinkConfiguration.m_angularLimitPositive = limitUpper;
                }
                else if (type == PhysX::ArticulationJointType::Prismatic)
                {
                    articulationLinkConfiguration.m_linearLimitLower = joint->limits->upper;
                    articulationLinkConfiguration.m_linearLimitUpper = joint->limits->lower;
                }
            }
            else
            {
                articulationLinkConfiguration.m_isLimited = false;
            }
        }
        return articulationLinkConfiguration;
    }

    ArticulationCfg& AddToArticulationConfig(ArticulationCfg& articulationLinkConfiguration, const urdf::InertialSharedPtr inertial)
    {
        if (!inertial)
        {
            return articulationLinkConfiguration;
        }
        articulationLinkConfiguration.m_solverPositionIterations =
            AZStd::max(articulationLinkConfiguration.m_solverPositionIterations, URDF::DefaultNumberPosSolver);
        articulationLinkConfiguration.m_solverVelocityIterations =
            AZStd::max(articulationLinkConfiguration.m_solverVelocityIterations, URDF::DefaultNumberVelSolver);

        articulationLinkConfiguration.m_mass = inertial->mass;
        articulationLinkConfiguration.m_centerOfMassOffset = URDF::TypeConversions::ConvertVector3(inertial->origin.position);

        if (!URDF::TypeConversions::ConvertQuaternion(inertial->origin.rotation).IsIdentity())
        { // There is a rotation component in URDF that we are not able to apply
            AZ_Warning("AddArticulationLink", false, "Ignoring URDF inertial origin rotation (no such field in rigid body configuration)");
        }
        return articulationLinkConfiguration;
    }

    void ArticulationsMaker::AddArticulationLink(const urdf::LinkSharedPtr link, AZ::EntityId entityId) const
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "No entity for id %s", entityId.ToString().c_str());

        AZ_TracePrintf("ArticulationsMaker", "Processing inertial for entity id: %s\n", entityId.ToString().c_str());
        PhysX::EditorArticulationLinkConfiguration articulationLinkConfiguration;

        articulationLinkConfiguration = AddToArticulationConfig(articulationLinkConfiguration, link->inertial);
        articulationLinkConfiguration = AddToArticulationConfig(articulationLinkConfiguration, link->parent_joint);

        entity->CreateComponent<PhysX::EditorArticulationLinkComponent>(articulationLinkConfiguration);
    }
} // namespace ROS2
