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
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/std/string/regex.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <string.h>

namespace ROS2
{

    bool Utils::WaitForAssetsToProcess(const AZStd::unordered_map<AZStd::string, AZ::IO::Path>& sourceAssetsPaths)
    {
        bool allAssetProcessed = false;
        bool assetProcessorFailed = false;
        auto loopStartTime = AZStd::chrono::system_clock::now();

        AZStd::chrono::seconds assetJobTimeout = AZStd::chrono::seconds(30);
        auto settingsRegistry = AZ::SettingsRegistry::Get();
        AZ::s64 loopTimeoutValue;
        if (settingsRegistry->Get(loopTimeoutValue, "/O3DE/ROS2/RobotImporter/AssetProcessorTimeoutInSeconds"))
        {
            assetJobTimeout = AZStd::chrono::seconds(loopTimeoutValue);
        }

        /* This loop waits until all of the assets are processed.
           There are three stop conditions: allAssetProcessed, assetProcessorFailed and a timeout.
           After all asset are processed the allAssetProcessed will be set to true.
           assetProcessorFailed will be set to true if the asset processor does not respond.
           The time out will break the loop if assetLoopTimeout is exceed. */
        while (!allAssetProcessed && !assetProcessorFailed)
        {
            auto loopTime = AZStd::chrono::system_clock::now();

            if (loopTime - loopStartTime > assetJobTimeout)
            {
                AZ_Warning("RobotImporterUtils", false, "Loop waiting for assets timed out");
                break;
            }

            allAssetProcessed = true;
            for (const auto& [AssetFileName, AssetFilePath] : sourceAssetsPaths)
            {
                if (AssetFilePath.empty())
                {
                    AZ_Warning("RobotImporterUtils", false, "WaitForAssetsToProcess got an empty filepath ");
                    continue;
                }
                using namespace AzToolsFramework;
                using namespace AzToolsFramework::AssetSystem;
                AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure();
                AssetSystemJobRequestBus::BroadcastResult(
                    result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, AssetFilePath.String(), true);

                if (!result.IsSuccess())
                {
                    assetProcessorFailed = true;
                    AZ_Error("RobotImporterUtils", false, "Asset System failed to reply with jobs infos");
                    break;
                }

                JobInfoContainer& allJobs = result.GetValue();
                for (const JobInfo& job : allJobs)
                {
                    if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
                    {
                        allAssetProcessed = false;
                    }
                }
            }

            if (allAssetProcessed && !assetProcessorFailed)
            {
                AZ_Printf("RobotImporterUtils", "All assets processed");
                return true;
            }
        }

        return false;
    }

    bool Utils::IsWheelURDFHeuristics(const sdf::Link* link)
    {
        const AZStd::regex wheel_regex("wheel[_]||[_]wheel");
        const AZStd::regex joint_regex("(?i)joint");
        const AZStd::string link_name(link->Name().c_str(), link->Name().size());
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
        if ((link->CollisionCount() == 0) || (link->VisualCount() == 0))
        {
            return false;
        }
        // Parent joint needs to be CONTINOUS
        // TODO: Figure out parent/child joints
        /*
        if (link->parent_joint && link->parent_joint->type == urdf::Joint::CONTINUOUS)
        {
            return true;
        }
        */
        return false;
    }

    AZ::Transform Utils::GetWorldTransformURDF(const sdf::Link* link, AZ::Transform t)
    {
        // TODO: Figure out parent/child links
        /*
        if (link->getParent() != nullptr)
        {
            t = URDF::TypeConversions::ConvertPose(link->parent_joint->parent_to_joint_origin_transform) * t;
            return GetWorldTransformURDF(link->getParent(), t);
        }
        */
        return t;
    }

    AZStd::unordered_map<AZStd::string, const sdf::Link*> Utils::GetAllLinks(const std::vector<const sdf::Link*>& childLinks)
    {
        AZStd::unordered_map<AZStd::string, const sdf::Link*> pointers;
        AZStd::function<void(const std::vector<const sdf::Link*>&)> link_visitor =
            [&](const std::vector<const sdf::Link*>& child_links) -> void
        {
            for (const sdf::Link* child_link : child_links)
            {
                AZStd::string link_name(child_link->Name().c_str(), child_link->Name().size());
                pointers[link_name] = child_link;
                // TODO: Figure out parent/child links
                //link_visitor(child_link->child_links);
            }
        };
        link_visitor(childLinks);
        return pointers;
    }

    AZStd::unordered_map<AZStd::string, const sdf::Joint*> Utils::GetAllJoints(const std::vector<const sdf::Link*>& childLinks)
    {
        AZStd::unordered_map<AZStd::string, const sdf::Joint*> joints;
        AZStd::function<void(const std::vector<const sdf::Link*>&)> link_visitor =
            [&](const std::vector<const sdf::Link*>& child_links) -> void
        {
            // TODO: Figure out parent/child links
            /*
            for (auto child_link : child_links)
            {
                const auto& joint = child_link->parent_joint;
                AZStd::string joint_name(joint->Name().c_str(), joint->N()ame.size());
                joints[joint_name] = joint;
                //link_visitor(child_link->child_links);
            }
            */
        };
        link_visitor(childLinks);
        return joints;
    }

    AZStd::unordered_set<AZStd::string> Utils::GetMeshesFilenames(const sdf::Root* rootLink, bool visual, bool colliders)
    {
        AZStd::unordered_set<AZStd::string> filenames;
        const auto addFilenameFromGeometry = [&filenames](const sdf::Geometry* geometry)
        {
            if (geometry->Type() == sdf::GeometryType::MESH)
            {
                auto pMesh = geometry->MeshShape();
                if (pMesh)
                {
                    filenames.insert(AZStd::string(pMesh->Uri().c_str(), pMesh->Uri().size()));
                }
            }
        };

        const auto processLink = [&addFilenameFromGeometry, visual, colliders](const sdf::Link* link)
        {
            if (visual)
            {
                for (uint64_t index = 0; index < link->VisualCount(); index++)
                {
                    addFilenameFromGeometry(link->VisualByIndex(index)->Geom());
                }
            }
            if (colliders)
            {
                for (uint64_t index = 0; index < link->CollisionCount(); index++)
                {
                    addFilenameFromGeometry(link->CollisionByIndex(index)->Geom());
                }
            }
        };

        AZStd::function<void(const std::vector<const sdf::Link*>&)> linkVisitor =
            [&](const std::vector<const sdf::Link*>& child_links) -> void
        {
            for (auto link : child_links)
            {
                processLink(link);
                // TODO: Figure out parent/child
                /*
                std::vector<const sdf::Link*> childVector(link->child_links.size());
                std::transform(
                    link->child_links.begin(),
                    link->child_links.end(),
                    childVector.begin(),
                    [](const sdf::Link* p)
                    {
                        return p;
                    });
                linkVisitor(childVector);
                */
            }
        };
        for (uint64_t index = 0; index < rootLink->Model()->LinkCount(); index++)
        {
            linkVisitor({ rootLink->Model()->LinkByIndex(index) });
        }
        return filenames;
    }

    AZStd::optional<AZ::IO::Path> GetResolvedPath(
        const AZ::IO::Path& packagePath, const AZ::IO::Path& unresolvedPath, const AZStd::function<bool(const AZStd::string&)>& fileExists)
    {
        AZ::IO::Path packageXmlCandite = packagePath / "package.xml";
        if (fileExists(packageXmlCandite.String()))
        {
            AZ::IO::Path resolvedPath = packagePath / unresolvedPath;
            if (fileExists(resolvedPath.String()))
            {
                return AZStd::optional<AZ::IO::Path>{ resolvedPath };
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
        for (AZ::IO::Path::iterator pathIt = begin; pathIt != end; pathIt++)
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
                        AZStd::advance(pathIt, 1);
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
                    const auto packagePath = GetPathFromSubPath(urdfProperPath.begin(), it);
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

    namespace Utils::SDFormat
    {
        AZStd::string GetPluginFilename(const sdf::Plugin& plugin)
        {
            const AZ::IO::Path path{ plugin.Filename().c_str() };
            return path.Filename().String();
        }

        AZStd::vector<AZStd::string> GetUnsupportedParams(
            const sdf::ElementPtr& rootElement, const AZStd::unordered_set<AZStd::string>& supportedParams)
        {
            AZStd::vector<AZStd::string> unsupportedParams;

            AZStd::function<void(const sdf::ElementPtr& elementPtr, const std::string& prefix)> elementVisitor =
                [&](const sdf::ElementPtr& elementPtr, const std::string& prefix) -> void
            {
                auto childPtr = elementPtr->GetFirstElement();

                AZStd::string prefixAz(prefix.c_str(), prefix.size());
                if (!childPtr && !prefixAz.empty() && !supportedParams.contains(prefixAz))
                {
                    unsupportedParams.push_back(prefixAz);
                }

                while (childPtr)
                {
                    if (childPtr->GetName() == "plugin")
                    {
                        break;
                    }

                    std::string currentName = prefix;
                    currentName.append(">");
                    currentName.append(childPtr->GetName());

                    elementVisitor(childPtr, currentName);
                    childPtr = childPtr->GetNextElement();
                }
            };

            elementVisitor(rootElement, "");

            return unsupportedParams;
        }

        bool IsPluginSupported(const sdf::Plugin& plugin, const AZStd::unordered_set<AZStd::string>& supportedPlugins)
        {
            return supportedPlugins.contains(GetPluginFilename(plugin));
        }
    } // namespace Utils::SDFormat

} // namespace ROS2
