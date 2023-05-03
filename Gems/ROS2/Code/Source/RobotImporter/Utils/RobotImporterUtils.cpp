/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporterUtils.h"
#include "TypeConversions.h"
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/std/string/regex.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>

namespace ROS2
{

    bool Utils::IsWheelURDFHeuristics(const urdf::LinkConstSharedPtr& link)
    {
        const AZStd::regex wheel_regex("wheel[_]||[_]wheel");
        const AZStd::regex joint_regex("(?i)joint");
        const AZStd::string link_name(link->name.c_str(), link->name.size());
        AZStd::smatch match;
        // Check if name is catchy for wheel
        if (!AZStd::regex_search(link_name, match, wheel_regex))
        {
            return false;
        }
        // The name should contain a joint word
        if (AZStd::regex_search(link_name, match, joint_regex))
        {
            return false;
        }
        // Wheels need to have collision and visuals
        if (!(link->collision && link->visual))
        {
            return false;
        }
        // Parent joint needs to be CONTINOUS
        if (link->parent_joint && link->parent_joint->type == urdf::Joint::CONTINUOUS)
        {
            return true;
        }
        return false;
    }

    AZ::Transform Utils::GetWorldTransformURDF(const urdf::LinkSharedPtr& link, AZ::Transform t)
    {
        if (link->getParent() != nullptr)
        {
            t = URDF::TypeConversions::ConvertPose(link->parent_joint->parent_to_joint_origin_transform) * t;
            return GetWorldTransformURDF(link->getParent(), t);
        }
        return t;
    }

    AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> Utils::GetAllLinks(const std::vector<urdf::LinkSharedPtr>& childLinks)
    {
        AZStd::unordered_map<AZStd::string, urdf::LinkSharedPtr> pointers;
        AZStd::function<void(const std::vector<urdf::LinkSharedPtr>&)> link_visitor =
            [&](const std::vector<urdf::LinkSharedPtr>& child_links) -> void
        {
            for (const urdf::LinkSharedPtr& child_link : child_links)
            {
                AZStd::string link_name(child_link->name.c_str(), child_link->name.size());
                pointers[link_name] = child_link;
                link_visitor(child_link->child_links);
            }
        };
        link_visitor(childLinks);
        return pointers;
    }

    AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> Utils::GetAllJoints(const std::vector<urdf::LinkSharedPtr>& childLinks)
    {
        AZStd::unordered_map<AZStd::string, urdf::JointSharedPtr> joints;
        AZStd::function<void(const std::vector<urdf::LinkSharedPtr>&)> link_visitor =
            [&](const std::vector<urdf::LinkSharedPtr>& child_links) -> void
        {
            for (auto child_link : child_links)
            {
                const auto& joint = child_link->parent_joint;
                AZStd::string joint_name(joint->name.c_str(), joint->name.size());
                joints[joint_name] = joint;
                link_visitor(child_link->child_links);
            }
        };
        link_visitor(childLinks);
        return joints;
    }

    AZStd::unordered_set<AZStd::string> Utils::GetMeshesFilenames(const urdf::LinkConstSharedPtr& rootLink, bool visual, bool colliders)
    {
        AZStd::unordered_set<AZStd::string> filenames;
        const auto addFilenameFromGeometry = [&filenames](const urdf::GeometrySharedPtr& geometry)
        {
            if (geometry->type == urdf::Geometry::MESH)
            {
                auto pMesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
                if (pMesh)
                {
                    filenames.insert(AZStd::string(pMesh->filename.c_str(), pMesh->filename.size()));
                }
            }
        };

        const auto processLink = [&addFilenameFromGeometry, visual, colliders](const urdf::Link& link)
        {
            if (visual)
            {
                for (auto& p : link.visual_array)
                {
                    addFilenameFromGeometry(p->geometry);
                }
            }
            if (colliders)
            {
                for (auto& p : link.collision_array)
                {
                    addFilenameFromGeometry(p->geometry);
                }
            }
        };

        AZStd::function<void(const std::vector<urdf::LinkConstSharedPtr>&)> linkVisitor =
            [&](const std::vector<urdf::LinkConstSharedPtr>& child_links) -> void
        {
            for (auto link : child_links)
            {
                processLink(*link);
                std::vector<urdf::LinkConstSharedPtr> childVector(link->child_links.size());
                std::transform(
                    link->child_links.begin(),
                    link->child_links.end(),
                    childVector.begin(),
                    [](const urdf::LinkSharedPtr& p)
                    {
                        return urdf::const_pointer_cast<urdf::Link>(p);
                    });
                linkVisitor(childVector);
            }
        };
        linkVisitor({ rootLink });
        return filenames;
    }

    AZStd::optional<AZ::IO::Path> GetResolvedPath(const AZ::IO::Path &packagePath,
                                                 const AZ::IO::Path &unresolvedPath,
                                                 const AZStd::function<bool(const AZStd::string&)>& fileExists)
    {
        AZ::IO::Path packageXmlCandite = packagePath / "package.xml";
        if (fileExists(packageXmlCandite.String()))
        {
            AZ::IO::Path resolvedPath = packagePath / unresolvedPath;
            if (fileExists(resolvedPath.String()))
            {
                return AZStd::optional<AZ::IO::Path>{resolvedPath};
            }
        }
        return AZStd::optional<AZ::IO::Path>{};
    }

    AZ::IO::Path GetPathFromSubPath(const AZ::IO::Path::const_iterator& begin, const AZ::IO::Path::const_iterator& end)
    {
        AZ::IO::Path subpath;
        if (begin == end)
        {
            return subpath;
        }
        for (AZ::IO::Path::iterator pathIt = begin;pathIt!=end;pathIt++)
        {
            subpath /= *pathIt;
        }
        return subpath;
    }

    /// Finds global path from URDF path
    AZStd::string Utils::ResolveURDFPath(
        AZStd::string unresolvedPath,
        const AZStd::string& urdfFilePath,
        const AZStd::string& amentPrefixPath,
        const AZStd::function<bool(const AZStd::string&)>& fileExists)
    {
        AZ_Printf("ResolveURDFPath", "ResolveURDFPath with %s\n", unresolvedPath.c_str());
        if (unresolvedPath.starts_with("package://"))
        {
            AZ::StringFunc::Replace(unresolvedPath, "package://", "", true, true);

            const AZ::IO::Path unresolvedProperPath(unresolvedPath);
            if (!unresolvedProperPath.empty())
            {
                const AZStd::string packageNameCandidate = unresolvedProperPath.begin()->String();
                AZStd::vector<AZStd::string> amentPathTokenized;
                AZ::StringFunc::Tokenize(amentPrefixPath, amentPathTokenized, ':');
                for (const auto& package : amentPathTokenized)
                {
                    if (package.ends_with(packageNameCandidate))
                    {
                        auto pathIt = unresolvedProperPath.begin();
                        AZStd::advance(pathIt,1);
                        if (pathIt != unresolvedProperPath.end())
                        {
                            AZ::IO::Path unresolvedPathStripped = GetPathFromSubPath(pathIt, unresolvedProperPath.end());

                            const AZ::IO::Path packagePath = AZ::IO::Path{ package } / "share";
                            auto resolvedPath =
                                GetResolvedPath(packagePath / AZ::IO::Path{ packageNameCandidate }, unresolvedPathStripped, fileExists);
                            if (resolvedPath.has_value())
                            {
                                AZ_Printf("ResolveURDFPath", "Resolved to using Ament to : %s\n", resolvedPath->String().c_str());
                                return resolvedPath->String();
                            }
                        }
                    }
                }
            }

            const AZ::IO::Path urdfProperPath(urdfFilePath);
            if (!urdfProperPath.empty())
            {
                auto it = --urdfProperPath.end();
                for (; it != urdfProperPath.begin(); it--)
                {
                    const auto packagePath = GetPathFromSubPath(urdfProperPath.begin(),it);
                    std::cout << "packagePath : " << packagePath.String().c_str() << std::endl;
                    const auto resolvedPath = GetResolvedPath(packagePath, unresolvedPath, fileExists);
                    if (resolvedPath.has_value())
                    {
                        AZ_Printf("ResolveURDFPath", "ResolveURDFPath with relative path to : %s\n", resolvedPath->String().c_str());
                        return resolvedPath->String();
                    }
                }
            }
            // No path available
            return "";
        }
        if (unresolvedPath.starts_with("file:///"))
        {
            // Paths that start with 'file:///' are absolute paths
            AZ::StringFunc::Replace(unresolvedPath, "file://", "", true, true);
            AZ_Printf("ResolveURDFPath", "ResolveURDFPath with absolute path to : %s\n", unresolvedPath.c_str());
            return unresolvedPath;
        }
        // seems to be relative path
        AZ::IO::Path relativePath(unresolvedPath);
        AZ::IO::Path urdfProperPath(urdfFilePath);
        AZ::IO::Path urdfParentPath = urdfProperPath.ParentPath();
        const AZ::IO::Path resolvedPath = urdfParentPath / relativePath;
        AZ_Printf("ResolveURDFPath", "ResolveURDFPath with relative path to : %s\n", unresolvedPath.c_str());
        return resolvedPath.String();
    }

} // namespace ROS2
