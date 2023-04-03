/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ArticulationsMaker.h"
#include <AzCore/Component/EntityId.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <Source/EditorArticulationLinkComponent.h>

namespace ROS2
{
    // Here is the recommended, minimal number of iterations for position and velocity solver.
    // It is needed since currently o3de default values are optimized for the gaming experience, not a simulation.
    constexpr AZ::u8 MinimalNumPosSolvArticulations = 40;
    constexpr AZ::u8 MinimalNumVelSolvArticulations = 10;

    void ArticulationsMaker::AddArticulationLink(urdf::LinkSharedPtr link, AZ::EntityId entityId) const
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        AZ_Assert(entity, "No entity for id %s", entityId.ToString().c_str());

        AZ_TracePrintf("ArticulationsMaker", "Processing inertial for entity id: %s\n", entityId.ToString().c_str());
        PhysX::EditorArticulationLinkConfiguration articulationLinkConfiguration;
        const auto inertial = link->inertial;
        if (inertial)
        {
            articulationLinkConfiguration.m_solverVelocityIterations = MinimalNumVelSolvArticulations;
            articulationLinkConfiguration.m_solverPositionIterations = MinimalNumPosSolvArticulations;
            articulationLinkConfiguration.m_mass = inertial->mass;
            articulationLinkConfiguration.m_centerOfMassOffset = URDF::TypeConversions::ConvertVector3(inertial->origin.position);

            if (!URDF::TypeConversions::ConvertQuaternion(inertial->origin.rotation).IsIdentity())
            { // There is a rotation component in URDF that we are not able to apply
                AZ_Warning(
                    "AddArticulationLink", false, "Ignoring URDF inertial origin rotation (no such field in rigid body configuration)");
            }
        }
        const auto joint = link->parent_joint;
        if (joint)
        {
            auto supportedArticulationType = SupportedJointTypes.find(joint->type);
            AZ_Warning(
                "ArticulationsMaker",
                supportedArticulationType != SupportedJointTypes.end(),
                "Articulations do not support type of URDF joint : %d, name of joint : %s",
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
                    }else if (type == PhysX::ArticulationJointType::Prismatic ){
                        articulationLinkConfiguration.m_linearLimitLower = joint->limits->upper;
                        articulationLinkConfiguration.m_linearLimitUpper = joint->limits->lower;
                    }
                }else{
                    articulationLinkConfiguration.m_isLimited = false;
                }
            }
        }

        entity->CreateComponent<PhysX::EditorArticulationLinkComponent>(articulationLinkConfiguration);
    }
} // namespace ROS2
