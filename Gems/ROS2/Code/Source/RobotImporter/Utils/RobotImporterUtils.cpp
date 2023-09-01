/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporterUtils.h"
#include <RobotImporter/Utils/ErrorUtils.h>
#include "TypeConversions.h"
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/std/string/regex.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <string.h>

namespace ROS2::Utils
{
    inline namespace Internal
    {
        bool FileExistsCall(const AZ::IO::PathView& filePath)
        {
            AZ::IO::FixedMaxPath pathStorage(filePath);
            return AZ::IO::SystemFile::Exists(pathStorage.c_str());
        };
    }

    bool IsWheelURDFHeuristics(const sdf::Model& model, const sdf::Link* link)
    {
        auto wheelMatcher = [](AZStd::string_view name)
        {
            // StringFunc matches are case-insensitive by default
            return AZ::StringFunc::StartsWith(name, "wheel_") ||
                AZ::StringFunc::EndsWith(name, "_wheel");
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
            if (const sdf::JointAxis* jointAxis = potentialWheelJoint->Axis(); jointAxis != nullptr)
            {
                using LimitType = decltype(jointAxis->Lower());
                // There should only be 1 element for URDF, however that will not be verified
                // in case this function is called on link from an SDF file
                isWheel = potentialWheelJoint->Type() == sdf::JointType::CONTINUOUS;
                isWheel = isWheel || (potentialWheelJoint->Type() == sdf::JointType::REVOLUTE
                    && jointAxis->Lower() == -AZStd::numeric_limits<LimitType>::infinity()
                    && jointAxis->Upper() == AZStd::numeric_limits<LimitType>::infinity());
            }
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
            AZStd::string poseErrorMessages = Utils::JoinSdfErrorsToString(poseResolveErrors);

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
                if (VisitLinksForModel(model) && m_recurseModels)
                {
                    // Nested model link are only visited if the joint visitor returns true
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        const sdf::Model* nestedModel = model.ModelByIndex(modelIndex);
                        if (nestedModel != nullptr)
                        {
                            if (!VisitLinksForModel(*nestedModel))
                            {
                                // Sibling nested model links are only visited
                                // if the joint visitor returns true
                                break;
                            }
                        }
                    }
                }
            }

        private:
            //! Returns success by default
            //! But an invoked visitor can return false to halt further iteration
            bool VisitLinksForModel(const sdf::Model& currentModel)
            {
                for (uint64_t linkIndex{}; linkIndex < currentModel.LinkCount(); ++linkIndex)
                {
                    const sdf::Link* link = currentModel.LinkByIndex(linkIndex);
                    if (link != nullptr)
                    {
                        if (!m_linkVisitorCB(*link))
                        {
                            return false;
                        }
                    }
                }

                return true;
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
                if (VisitJointsForModel(model) && m_recurseModels)
                {
                    // Nested model joints are only visited if the joint visitor returns true
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        const sdf::Model* nestedModel =  model.ModelByIndex(modelIndex);
                        if (nestedModel != nullptr)
                        {
                            if (!VisitJointsForModel(*nestedModel))
                            {
                                // Sibling nested model joints are only visited
                                // if the joint visitor returns true
                                break;
                            }
                        }
                    }
                }
            }

        private:
            bool VisitJointsForModel(const sdf::Model& currentModel)
            {
                for (uint64_t jointIndex{}; jointIndex < currentModel.JointCount(); ++jointIndex)
                {
                    const sdf::Joint* joint = currentModel.JointByIndex(jointIndex);
                    // Skip any joints whose parent and child link references
                    // don't have an actual sdf::link in the parsed model
                    if (joint != nullptr && currentModel.LinkNameExists(joint->ParentName())
                        && currentModel.LinkByName(joint->ChildName()))
                    {
                        if (!m_jointVisitorCB(*joint))
                        {
                            return false;
                        }
                    }
                }

                return true;
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
            AZStd::string linkName(link.Name().c_str(), link.Name().size());
            links.insert_or_assign(AZStd::move(linkName), &link);
            return true;
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
            AZStd::string jointName(joint.Name().c_str(), joint.Name().size());
            joints.insert_or_assign(AZStd::move(jointName), &joint);
            return true;
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
            if (AZStd::string_view jointChildName{ joint.ChildName().c_str(), joint.ChildName().size() };
                jointChildName == linkName)
            {
                joints.emplace_back(&joint);
            }

            return true;
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
            if (AZStd::string_view jointParentName{ joint.ParentName().c_str(), joint.ParentName().size() };
                jointParentName == linkName)
            {
                joints.emplace_back(&joint);
            }

            return true;
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

    /// Finds global path from URDF/SDF path
    AZ::IO::Path ResolveAssetPath(
        AZ::IO::Path unresolvedPath,
        const AZ::IO::PathView& baseFilePath,
        AZStd::string_view amentPrefixPath,
        const SdfAssetBuilderSettings& settings,
        const FileExistsCB& fileExistsCB)
    {
        AZ_Printf("ResolveAssetPath", "ResolveAssetPath with %s\n", unresolvedPath.c_str());

        const auto& pathResolverSettings = settings.m_resolverSettings;

        // Parse the AMENT_PREFIX_PATH environment variable if the settings enable using it for path resolution.
        // If not, leave the set of paths empty.
        AZStd::vector<AZ::IO::Path> amentPrefixPaths;
        if (pathResolverSettings.m_useAmentPrefixPath)
        {
            // Split the AMENT_PREFIX_PATH into multiple paths
            auto AmentPrefixPathVisitor = [&amentPrefixPaths](
                AZStd::string_view prefixPath)
            {
                amentPrefixPaths.push_back(prefixPath);
            };
            // Note this code only works on Unix platforms
            // For Windows this will not work as the drive letter has a colon in it (C:\)
            AZ::StringFunc::TokenizeVisitor(amentPrefixPath, AmentPrefixPathVisitor, ':');
        }

        // Append all ancestor directories from the root file to the candidate replacement paths if the settings enable using
        // them for path resolution and the root file isn't empty.
        AZStd::vector<AZ::IO::Path> ancestorPaths;
        if (!baseFilePath.empty() && pathResolverSettings.m_useAncestorPaths)
        {
            // The first time through this loop, fileAncestorPath contains the full file name ('/a/b/c.sdf') so
            // ParentPath() will return the path containing the file ('/a/b'). Each iteration will walk up the path
            // to the root, including the root, before stopping ('/a', '/').
            AZ::IO::Path fileAncestorPath = baseFilePath;
            do
            {
                fileAncestorPath = fileAncestorPath.ParentPath();
                ancestorPaths.emplace_back(fileAncestorPath);
            } while (fileAncestorPath != fileAncestorPath.RootPath());
        }

        // Loop through each prefix in the builder settings and attempt to resolve it.
        // TODO: also loop through each replacement path?
        for (const auto& [prefix, replacements] : pathResolverSettings.m_uriPrefixMap)
        {
            // Note this is a case-sensitive check to match the exact URI scheme
            // If that is not desired, then this code should be updated to read
            // a value from the Setting Registry indicating whether the uriPrefix matching
            // should be case sensitive
            constexpr bool uriPrefixMatchCaseSensitive = true;
            // If the path doesn't start with the given prefix, move on to the next prefix
            if (!AZ::StringFunc::StartsWith(unresolvedPath.Native(), prefix, uriPrefixMatchCaseSensitive))
            {
                continue;
            }

            // Strip the number of characters from the Uri scheme from beginning of the path
            AZ::IO::PathView strippedUriPath = AZ::IO::PathView(unresolvedPath).Native().substr(prefix.size());

            // If the stripped URI path is empty, there is nothing to resolve
            if (strippedUriPath.empty())
            {
                AZ_Trace("ResolveAssetPath", R"(Resolved Path is empty: "%.*s" -> "")" "\n", AZ_PATH_ARG(unresolvedPath));
                return "";
            }

            // If the remaining path is an absolute path, simply return whether or not it exists.
            if (strippedUriPath.IsAbsolute())
            {
                if (fileExistsCB(strippedUriPath))
                {
                    AZ_Trace("ResolveAssetPath", R"(Resolved Absolute Path: "%.*s" -> "%.*s")" "\n", 
                        AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(strippedUriPath));
                    return strippedUriPath;
                }
                else
                {
                    AZ_Trace("ResolveAssetPath", R"(Failed to resolve Absolute Path: "%.*s" -> "%.*s")" "\n", 
                        AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(strippedUriPath));
                    return "";
                }
            }

            // Check to see if the relative part of the URI path refers to a location
            // within each <ament prefix path>/share directory
            for (const AZ::IO::Path& amentPrefixPath : amentPrefixPaths)
            {
                auto pathIter = strippedUriPath.begin();
                AZ::IO::PathView packageName = *pathIter;
                const AZ::IO::Path amentSharePath = amentPrefixPath / "share";
                const AZ::IO::Path packageManifestPath = amentSharePath / packageName / "package.xml";

                // Given a path like 'ambulance/meshes/model.stl', it will be considered a match if
                // <ament prefix path>/share/ambulance/package.xml exists and 
                // <ament prefix path>/share/ambulance/meshes/model.stl exists.
                if (const AZ::IO::Path candidateResolvedPath = amentSharePath / strippedUriPath;
                    fileExistsCB(packageManifestPath) && fileExistsCB(candidateResolvedPath))
                {
                    AZ_Trace("ResolveAssetPath", R"(Resolved using AMENT_PREFIX_PATH: "%.*s" -> "%.*s")" "\n", 
                        AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(candidateResolvedPath));
                    return candidateResolvedPath;
                }
            }

            // The URI path cannot be resolved within the any ament prefix path,
            // so try the directory containing the root file as well as any of its parent directories
            for (const AZ::IO::Path& ancestorPath : ancestorPaths)
            {
                if (const AZ::IO::Path candidateResolvedPath = ancestorPath / strippedUriPath;
                    fileExistsCB(candidateResolvedPath))
                {
                    AZ_Trace("ResolveAssetPath", R"(Resolved using ancestor paths: "%.*s" -> "%.*s")" "\n", 
                        AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(candidateResolvedPath));
                    return candidateResolvedPath;
                }
            }
        }


        // At this point, the path has no identified URI prefix. If it's an absolute path, try to locate and return it.
        // Otherwise, return an empty path as an error.
        if (unresolvedPath.IsAbsolute())
        {
            if (fileExistsCB(unresolvedPath))
            {
                AZ_Trace("ResolveAssetPath", R"(Resolved Absolute Path: "%.*s")" "\n", 
                    AZ_PATH_ARG(unresolvedPath));
                return unresolvedPath;
            }
            else
            {
                AZ_Trace("ResolveAssetPath", R"(Failed to resolve Absolute Path: "%.*s")" "\n", 
                    AZ_PATH_ARG(unresolvedPath));
                return "";
            }
        }

        // The path is a relative path, so use the directory containing the base URDF/SDF file as the root path,
        // and if the file can be found successfully, return the path. Otherwise, return an empty path as an error.
        const AZ::IO::Path relativePath = AZ::IO::Path(baseFilePath.ParentPath()) / unresolvedPath;

        if (fileExistsCB(relativePath))
        {
            AZ_Trace("ResolveAssetPath", R"(Resolved Relative Path: "%.*s" -> "%.*s")" "\n", 
                AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(relativePath));
            return relativePath;
        }

        AZ_Trace("ResolveAssetPath", R"(Failed to resolve Relative Path: "%.*s" -> "%.*s")" "\n", 
            AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(relativePath));
        return "";
    }

    AZStd::fixed_string<AmentPrefixPathMaxSize> GetAmentPrefixPath()
    {
        // Support reading the AMENT_PREFIX_PATH environment variable on Unix/Windows platforms
        auto StoreAmentPrefixPath = [](char* buffer, size_t size) -> size_t
        {
            auto getEnvOutcome = AZ::Utils::GetEnv(AZStd::span(buffer, size), "AMENT_PREFIX_PATH");
            return getEnvOutcome ? getEnvOutcome.GetValue().size() : 0;
        };
        AZStd::fixed_string<AmentPrefixPathMaxSize> amentPrefixPath;
        amentPrefixPath.resize_and_overwrite(amentPrefixPath.capacity(), StoreAmentPrefixPath);
        AZ_Error("UrdfAssetMap", !amentPrefixPath.empty(), "AMENT_PREFIX_PATH is not found.");

        return amentPrefixPath;
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

    sdf::ParserConfig CreateSdfParserConfigFromSettings(const SdfAssetBuilderSettings& settings)
    {
        sdf::ParserConfig sdfConfig;

        sdfConfig.URDFSetPreserveFixedJoint(settings.m_urdfPreserveFixedJoints);

        // Fill in the URI resolution with the supplied prefix mappings.
        for (auto& entry : settings.m_resolverSettings.m_uriPrefixMap)
        {
            std::string uriPath;
            for(auto& path : entry.second)
            {
                if (!uriPath.empty())
                {
                    uriPath.append(std::string(":"));
                }

                uriPath.append(std::string(path.c_str(), path.size()));
            }
            if (!entry.first.empty() && !uriPath.empty())
            {
                std::string uriPrefix(entry.first.c_str(), entry.first.size());
                sdfConfig.AddURIPath(uriPrefix, uriPath);
                AZ_Info("SdfParserConfig", "Added URI mapping '%s' -> '%s'", uriPrefix.c_str(), uriPath.c_str());
            }
        }

        // If any files couldn't be found using our supplied prefix mappings, this callback will get called.
        // Print a warning for any missing files.
        sdfConfig.SetFindCallback([](const std::string &fileName) -> std::string
        {
            AZ_Warning("SdfParserConfig", false, "SDF SetFindCallback called with '%s'", fileName.c_str());
            return fileName;
        });

        return sdfConfig;
    }
} // namespace ROS2::Utils::SDFormat
