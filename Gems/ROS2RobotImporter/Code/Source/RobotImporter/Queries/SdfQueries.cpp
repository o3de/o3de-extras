/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotImporter/Queries/SdfQueries.h>
#include <RobotImporter/Queries/SdfVisitors.h>

namespace ROS2RobotImporter::Utils
{
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
