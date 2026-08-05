/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotImporter/Sdf/SdfHeuristics.h>

#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/Sdf/SdfQueries.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include <RobotImporter/Utils/TypeConversions.h>

namespace ROS2RobotImporter::Utils
{
    bool IsWheelHeuristics(const sdf::Model& model, const sdf::Link* link)
    {
        auto wheelMatcher = [](AZStd::string_view name)
        {
            // StringFunc matches are case-insensitive by default
            return AZ::StringFunc::Contains(name, "wheel");
        };

        const AZStd::string linkName(link->Name().c_str(), link->Name().size());
        // Check if link name is catchy for wheel
        if (!wheelMatcher(linkName))
        {
            return false;
        }

        // Wheels need to have collision and visuals
        if ((link->CollisionCount() == 0) || (link->VisualCount() == 0))
        {
            return false;
        }

        // When this link is a child, the parent link joint needs to be CONTINUOUS
        AZStd::vector<const sdf::Joint*> joints = GetJointsForChildLink(model, linkName, true);

        // URDFs only have a single parent
        // This is explained in the Pose frame semantics tutorial for sdformat
        // http://sdformat.org/tutorials?tut=pose_frame_semantics&ver=1.5#parent-frames-in-urdf

        // The SDF URDF parser converts continuous joints to revolute joints with a limit
        // of -infinity to +infinity
        // https://github.com/gazebosim/sdformat/blob/sdf13/src/parser_urdf.cc#L3009-L3039
        bool isWheel{};
        if (!joints.empty())
        {
            const sdf::Joint* potentialWheelJoint = joints.front();
            if (const sdf::JointAxis* jointAxis = potentialWheelJoint->Axis(); jointAxis != nullptr)
            {
                using LimitType = decltype(jointAxis->Lower());
                // There should only be 1 element for URDF, however that will not be verified
                // in case this function is called on link from an SDF file
                isWheel = potentialWheelJoint->Type() == sdf::JointType::CONTINUOUS;
                isWheel = isWheel ||
                    (potentialWheelJoint->Type() == sdf::JointType::REVOLUTE &&
                     jointAxis->Lower() == -AZStd::numeric_limits<LimitType>::infinity() &&
                     jointAxis->Upper() == AZStd::numeric_limits<LimitType>::infinity());
            }
        }

        return isWheel;
    }

    AZ::Transform GetLocalTransform(const sdf::SemanticPose& semanticPose, AZ::Transform t)
    {
        // Determine if the pose is relative to another link
        // See doxygen at
        // http://osrf-distributions.s3.amazonaws.com/sdformat/api/13.2.0/classsdf_1_1SDF__VERSION__NAMESPACE_1_1Link.html#a011d84b31f584938d89ac6b8c8a09eb3

        gz::math::Pose3d resolvedPose;
        if (sdf::Errors poseResolveErrors = semanticPose.Resolve(resolvedPose); !poseResolveErrors.empty())
        {
            AZStd::string poseErrorMessages = Utils::JoinSdfErrorsToString(poseResolveErrors);

            AZ_Error("RobotImporter", false, R"(Failed to get world transform. Errors: "%s")", poseErrorMessages.c_str());
            return {};
        }

        const AZ::Transform localTransform = Utils::TypeConversions::ConvertPose(resolvedPose);
        const AZ::Transform resolvedTransform = localTransform * t;
        return resolvedTransform;
    }
} // namespace ROS2RobotImporter::Utils
