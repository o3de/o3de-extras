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

namespace ROS2::Utils
{
    bool WaitForAssetsToProcess(const AZStd::unordered_map<AZStd::string, AZ::IO::Path>& sourceAssetsPaths)
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

    bool IsWheelURDFHeuristics(const sdf::Model& model, const sdf::Link* link)
    {
        const AZStd::regex wheelRegex("wheel[_]||[_]wheel");
        const AZStd::regex jointRegex("(?i)joint");
        const AZStd::string linkName(link->Name().c_str(), link->Name().size());
        AZStd::smatch match;
        // Check if name is catchy for wheel
        if (!AZStd::regex_search(linkName, match, wheelRegex))
        {
            return false;
        }
        // The name should contain a joint word
        if (AZStd::regex_search(linkName, match, jointRegex))
        {
            return false;
        }
        // Wheels need to have collision and visuals
        if ((link->CollisionCount() == 0) || (link->VisualCount() == 0))
        {
            return false;
        }

        // When this link is a child, the parent link joint needs to be CONTINUOUS
        AZStd::vector<const sdf::Joint*> joints = GetJointsForChildLink(model,
            linkName, true);

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
            using LimitType = decltype(potentialWheelJoint->Axis()->Lower());
            // There should only be 1 element for URDF, however that will not be verified
            // in case this function is called on link from an SDF file
            isWheel = potentialWheelJoint->Type() == sdf::JointType::CONTINUOUS;
            isWheel = isWheel || (potentialWheelJoint->Type() == sdf::JointType::REVOLUTE
                && potentialWheelJoint->Axis()->Lower() == -AZStd::numeric_limits<LimitType>::infinity()
                && potentialWheelJoint->Axis()->Upper() == AZStd::numeric_limits<LimitType>::infinity());
        }

        return isWheel;
    }

    AZ::Transform GetWorldTransformURDF(const sdf::Link* link, AZ::Transform t)
    {
        // Determine if the pose is relative to another link
        // See doxygen at http://osrf-distributions.s3.amazonaws.com/sdformat/api/13.2.0/classsdf_1_1SDF__VERSION__NAMESPACE_1_1Link.html#a011d84b31f584938d89ac6b8c8a09eb3

        sdf::SemanticPose linkSemanticPos = link->SemanticPose();
        gz::math::Pose3d resolvedPose;

        if (sdf::Errors poseResolveErrors = linkSemanticPos.Resolve(resolvedPose);
            !poseResolveErrors.empty())
        {
            AZStd::string poseErrorMessages;
            for (const sdf::Error& error : poseResolveErrors)
            {
                AZStd::string errorMessage = AZStd::string::format("ErrorCode=%d", static_cast<int32_t>(error.Code()));
                errorMessage += AZStd::string::format(", Message=%s", error.Message().c_str());
                if (error.LineNumber().has_value())
                {
                    errorMessage += AZStd::string::format(", Line=%d", error.LineNumber().value());
                }

                poseErrorMessages += errorMessage;
                poseErrorMessages += '\n';
            }

            AZ_Error("RobotImporter", false, R"(Failed to get world transform for link %s. Errors: "%s")",
                link->Name().c_str(), poseErrorMessages.c_str());
            return {};
        }

        const AZ::Transform linkTransform = URDF::TypeConversions::ConvertPose(resolvedPose);
        const AZ::Transform resolvedTransform = linkTransform * t;
        return resolvedTransform;
    }

    void VisitLinks(const sdf::Model& sdfModel, const LinkVisitorCallback& linkVisitorCB,
        bool visitNestedModelLinks)
    {
        // Function object which can visit all links of a model
        // Optionally it supports recursing nested models to visit their links as well
        struct VisitLinksForNestedModels_fn
        {
            void operator()(const sdf::Model& model)
            {
                VisitLinksForModel(model);
                if (m_recurseModels)
                {
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        const sdf::Model* nestedModel =  model.ModelByIndex(modelIndex);
                        if (nestedModel != nullptr)
                        {
                            VisitLinksForModel(*nestedModel);
                        }
                    }
                }
            }

        private:
            void VisitLinksForModel(const sdf::Model& currentModel)
            {
                for (uint64_t linkIndex{}; linkIndex < currentModel.LinkCount(); ++linkIndex)
                {
                    const sdf::Link* link = currentModel.LinkByIndex(linkIndex);
                    if (link != nullptr)
                    {
                        m_linkVisitorCB(*link);
                    }
                }
            }

        public:
            LinkVisitorCallback m_linkVisitorCB;
            bool m_recurseModels{};
        };

        VisitLinksForNestedModels_fn VisitLinksForNestedModels{};
        VisitLinksForNestedModels.m_linkVisitorCB = linkVisitorCB;
        VisitLinksForNestedModels.m_recurseModels = visitNestedModelLinks;
        VisitLinksForNestedModels(sdfModel);
    }

    void VisitJoints(const sdf::Model& sdfModel, const JointVisitorCallback& jointVisitorCB,
        bool visitNestedModelJoints)
    {
        // Function object which can visit all joints of a model
        // Optionally it supports recursing nested models to visit their joints as well
        struct VisitJointsForNestedModels_fn
        {
            void operator()(const sdf::Model& model)
            {
                VisitJointsForModel(model);
                if (m_recurseModels)
                {
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        const sdf::Model* nestedModel =  model.ModelByIndex(modelIndex);
                        if (nestedModel != nullptr)
                        {
                            VisitJointsForModel(*nestedModel);
                        }
                    }
                }
            }

        private:
            void VisitJointsForModel(const sdf::Model& currentModel)
            {
                for (uint64_t jointIndex{}; jointIndex < currentModel.JointCount(); ++jointIndex)
                {
                    const sdf::Joint* joint = currentModel.JointByIndex(jointIndex);
                    if (joint != nullptr)
                    {
                        m_jointVisitorCB(*joint);
                    }
                }
            }

        public:
            JointVisitorCallback m_jointVisitorCB;
            bool m_recurseModels{};
        };

        VisitJointsForNestedModels_fn VisitJointsForNestedModels{};
        VisitJointsForNestedModels.m_jointVisitorCB = jointVisitorCB;
        VisitJointsForNestedModels.m_recurseModels = visitNestedModelJoints;
        VisitJointsForNestedModels(sdfModel);
    }

    AZStd::unordered_map<AZStd::string, const sdf::Link*> GetAllLinks(const sdf::Model& sdfModel,
        bool gatherNestedModelLinks)
    {
        using LinkMap = AZStd::unordered_map<AZStd::string, const sdf::Link*>;
        LinkMap links;
        auto GatherLinks = [&links](const sdf::Link& link)
        {
            AZStd::string_view linkName(link.Name().c_str(), link.Name().size());
            links.insert_or_assign(linkName, &link);
        };

        VisitLinks(sdfModel, GatherLinks, gatherNestedModelLinks);
        return links;
    }

    AZStd::unordered_map<AZStd::string, const sdf::Joint*> GetAllJoints(const sdf::Model& sdfModel,
        bool gatherNestedModelJoints)
    {
        using JointMap = AZStd::unordered_map<AZStd::string, const sdf::Joint*>;
        JointMap joints;
        auto GatherJoints = [&joints](const sdf::Joint& joint)
        {
            AZStd::string_view jointName(joint.Name().c_str(), joint.Name().size());
            joints.insert_or_assign(jointName, &joint);
        };

        VisitJoints(sdfModel, GatherJoints, gatherNestedModelJoints);
        return joints;
    }

    AZStd::vector<const sdf::Joint*> GetJointsForChildLink(const sdf::Model& sdfModel, AZStd::string_view linkName,
        bool gatherNestedModelJoints)
    {
        using JointVector = AZStd::vector<const sdf::Joint*>;
        JointVector joints;
        auto GatherJointsWhereLinkIsChild = [&joints, linkName](const sdf::Joint& joint)
        {
            AZStd::string_view jointChildName{ joint.ChildName().c_str(), joint.ChildName().size() };
            if (jointChildName == linkName)
            {
                joints.emplace_back(&joint);
            }
        };

        VisitJoints(sdfModel, GatherJointsWhereLinkIsChild, gatherNestedModelJoints);
        return joints;
    }

    AZStd::vector<const sdf::Joint*> GetJointsForParentLink(const sdf::Model& sdfModel, AZStd::string_view linkName,
        bool gatherNestedModelJoints)
    {
        using JointVector = AZStd::vector<const sdf::Joint*>;
        JointVector joints;
        auto GatherJointsWhereLinkIsParent = [&joints, linkName](const sdf::Joint& joint)
        {
            AZStd::string_view jointParentName{ joint.ParentName().c_str(), joint.ParentName().size() };
            if (jointParentName == linkName)
            {
                joints.emplace_back(&joint);
            }
        };

        VisitJoints(sdfModel, GatherJointsWhereLinkIsParent, gatherNestedModelJoints);
        return joints;
    }

    AZStd::unordered_set<AZStd::string> GetMeshesFilenames(const sdf::Root* root, bool visual, bool colliders)
    {
        const sdf::Model* model = root != nullptr ? root->Model() : nullptr;
        if (model == nullptr)
        {
            return {};
        }

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

        for (uint64_t index = 0; index < model->LinkCount(); index++)
        {
            processLink(model->LinkByIndex(index));
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
    AZStd::string ResolveURDFPath(
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

} // namespace ROS2::Utils

namespace ROS2::Utils::SDFormat
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
} // namespace ROS2::Utils::SDFormat
