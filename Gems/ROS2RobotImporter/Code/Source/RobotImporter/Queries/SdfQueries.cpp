/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/StringFunc/StringFunc.h>
#include <RobotImporter/Queries/SdfQueries.h>
#include <RobotImporter/Queries/SdfVisitors.h>
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

    AZStd::unordered_map<AZStd::string, const sdf::Link*> GetAllLinks(const sdf::Model& sdfModel, bool gatherNestedModelLinks)
    {
        using LinkMap = AZStd::unordered_map<AZStd::string, const sdf::Link*>;
        LinkMap links;
        auto GatherLinks = [&links](const sdf::Link& link, const ModelStack& modelStack)
        {
            std::string fullyQualifiedLinkName;
            // Prepend the Model names to the link name using the Name Scoping support in libsdformat
            // http://sdformat.org/tutorials?tut=composition_proposal#1-3-name-scoping-and-cross-referencing
            for (const sdf::Model& model : modelStack)
            {
                fullyQualifiedLinkName = sdf::JoinName(fullyQualifiedLinkName, model.Name());
            }
            fullyQualifiedLinkName = sdf::JoinName(fullyQualifiedLinkName, link.Name());

            AZStd::string azLinkName(fullyQualifiedLinkName.c_str(), fullyQualifiedLinkName.size());
            links.insert_or_assign(AZStd::move(azLinkName), &link);
            return true;
        };

        VisitLinks(sdfModel, GatherLinks, gatherNestedModelLinks);
        return links;
    }

    AZStd::unordered_map<AZStd::string, const sdf::Joint*> GetAllJoints(const sdf::Model& sdfModel, bool gatherNestedModelJoints)
    {
        using JointMap = AZStd::unordered_map<AZStd::string, const sdf::Joint*>;
        JointMap joints;
        auto GatherJoints = [&joints](const sdf::Joint& joint, const ModelStack& modelStack)
        {
            std::string fullyQualifiedJointName;
            // Prepend the Model names to the joint name using the Name Scoping support in libsdformat
            // http://sdformat.org/tutorials?tut=composition_proposal#1-3-name-scoping-and-cross-referencing
            for (const sdf::Model& model : modelStack)
            {
                fullyQualifiedJointName = sdf::JoinName(fullyQualifiedJointName, model.Name());
            }
            fullyQualifiedJointName = sdf::JoinName(fullyQualifiedJointName, joint.Name());

            AZStd::string azJointName(fullyQualifiedJointName.c_str(), fullyQualifiedJointName.size());
            joints.insert_or_assign(AZStd::move(azJointName), &joint);
            return true;
        };

        VisitJoints(sdfModel, GatherJoints, gatherNestedModelJoints);
        return joints;
    }

    AZStd::vector<const sdf::Joint*> GetJointsForChildLink(
        const sdf::Model& sdfModel, AZStd::string_view linkName, bool gatherNestedModelJoints)
    {
        using JointVector = AZStd::vector<const sdf::Joint*>;
        JointVector joints;
        auto GatherJointsWhereLinkIsChild = [&joints, linkName](const sdf::Joint& joint, const ModelStack& modelStack)
        {
            if (AZStd::string_view jointChildName{ joint.ChildName().c_str(), joint.ChildName().size() }; jointChildName == linkName)
            {
                joints.emplace_back(&joint);
            }

            return true;
        };

        VisitJoints(sdfModel, GatherJointsWhereLinkIsChild, gatherNestedModelJoints);
        return joints;
    }

    AZStd::vector<const sdf::Joint*> GetJointsForParentLink(
        const sdf::Model& sdfModel, AZStd::string_view linkName, bool gatherNestedModelJoints)
    {
        using JointVector = AZStd::vector<const sdf::Joint*>;
        JointVector joints;
        auto GatherJointsWhereLinkIsParent = [&joints, linkName](const sdf::Joint& joint, const ModelStack& modelStack)
        {
            if (AZStd::string_view jointParentName{ joint.ParentName().c_str(), joint.ParentName().size() }; jointParentName == linkName)
            {
                joints.emplace_back(&joint);
            }

            return true;
        };

        VisitJoints(sdfModel, GatherJointsWhereLinkIsParent, gatherNestedModelJoints);
        return joints;
    }
} // namespace ROS2RobotImporter::Utils
