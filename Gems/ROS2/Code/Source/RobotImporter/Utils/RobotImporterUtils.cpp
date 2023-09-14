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
#include <RobotImporter/Utils/ErrorUtils.h>
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
    } // namespace Internal

    bool IsWheelURDFHeuristics(const sdf::Model& model, const sdf::Link* link)
    {
        auto wheelMatcher = [](AZStd::string_view name)
        {
            // StringFunc matches are case-insensitive by default
            return AZ::StringFunc::StartsWith(name, "wheel_") || AZ::StringFunc::EndsWith(name, "_wheel");
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

    AZ::Transform GetWorldTransformURDF(const sdf::Link* link, AZ::Transform t)
    {
        // Determine if the pose is relative to another link
        // See doxygen at
        // http://osrf-distributions.s3.amazonaws.com/sdformat/api/13.2.0/classsdf_1_1SDF__VERSION__NAMESPACE_1_1Link.html#a011d84b31f584938d89ac6b8c8a09eb3

        sdf::SemanticPose linkSemanticPos = link->SemanticPose();
        gz::math::Pose3d resolvedPose;

        if (sdf::Errors poseResolveErrors = linkSemanticPos.Resolve(resolvedPose); !poseResolveErrors.empty())
        {
            AZStd::string poseErrorMessages = Utils::JoinSdfErrorsToString(poseResolveErrors);

            AZ_Error(
                "RobotImporter",
                false,
                R"(Failed to get world transform for link %s. Errors: "%s")",
                link->Name().c_str(),
                poseErrorMessages.c_str());
            return {};
        }

        const AZ::Transform linkTransform = URDF::TypeConversions::ConvertPose(resolvedPose);
        const AZ::Transform resolvedTransform = linkTransform * t;
        return resolvedTransform;
    }

    void VisitLinks(const sdf::Model& sdfModel, const LinkVisitorCallback& linkVisitorCB, bool visitNestedModelLinks)
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

    void VisitJoints(const sdf::Model& sdfModel, const JointVisitorCallback& jointVisitorCB, bool visitNestedModelJoints)
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
                        const sdf::Model* nestedModel = model.ModelByIndex(modelIndex);
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
                    if (joint != nullptr && currentModel.LinkNameExists(joint->ParentName()) && currentModel.LinkByName(joint->ChildName()))
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

    AZStd::unordered_map<AZStd::string, const sdf::Link*> GetAllLinks(const sdf::Model& sdfModel, bool gatherNestedModelLinks)
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

    AZStd::unordered_map<AZStd::string, const sdf::Joint*> GetAllJoints(const sdf::Model& sdfModel, bool gatherNestedModelJoints)
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

    AZStd::vector<const sdf::Joint*> GetJointsForChildLink(
        const sdf::Model& sdfModel, AZStd::string_view linkName, bool gatherNestedModelJoints)
    {
        using JointVector = AZStd::vector<const sdf::Joint*>;
        JointVector joints;
        auto GatherJointsWhereLinkIsChild = [&joints, linkName](const sdf::Joint& joint)
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
        auto GatherJointsWhereLinkIsParent = [&joints, linkName](const sdf::Joint& joint)
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

    //! Provides overloads for comparison operators for the VisitModelResponse enum
    AZ_DEFINE_ENUM_RELATIONAL_OPERATORS(VisitModelResponse);

    void VisitModels(const sdf::Root& sdfRoot, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels)
    {
        // Function object which can visit all models in an SDF document
        // Optionally it supports recursing nested models as well
        struct VisitModelsForNestedModels_fn
        {
            VisitModelResponse operator()(const sdf::Model& model)
            {
                // The VisitModelResponse enum value is used to filter out
                // less callbacks the higher the value grows.
                // So any values above VisitNestedAndSiblings will not visit nested models
                VisitModelResponse visitResponse = m_modelVisitorCB(model);

                if (m_recurseModels && visitResponse == VisitModelResponse::VisitNestedAndSiblings)
                {
                    // Nested models are only visited if the model visitor returns VisitNestedAndSiblings
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        if (const sdf::Model* nestedModel = model.ModelByIndex(modelIndex); nestedModel != nullptr)
                        {
                            if (VisitModelResponse nestedVisitResponse = operator()(*nestedModel);
                                nestedVisitResponse >= VisitModelResponse::Stop)
                            {
                                // Visiting of the nested model has returned Stop, so halt any sibling model visitation
                                break;
                            }
                        }
                    }
                }

                return visitResponse;
            }

            VisitModelResponse operator()(const sdf::World& world)
            {
                // Nested model are only visited if the model visitor returns true
                for (uint64_t modelIndex{}; modelIndex < world.ModelCount(); ++modelIndex)
                {
                    if (const sdf::Model* model = world.ModelByIndex(modelIndex); model != nullptr)
                    {
                        // Delegate to the sdf::Model call operator overload to visit nested models
                        // Stop visited the world's <model> children if any children return Stop
                        if (VisitModelResponse visitResponse = operator()(*model); visitResponse >= VisitModelResponse::Stop)
                        {
                            return visitResponse;
                        }
                    }
                }

                // By default visit any sibling worlds' models if visitation doesn't return Stop
                return VisitModelResponse::VisitNestedAndSiblings;
            }

            void operator()(const sdf::Root& root)
            {
                // Visit the root <model> tag if one exist
                VisitModelResponse modelVisitResponse = VisitModelResponse::VisitNestedAndSiblings;
                if (const sdf::Model* model = root.Model(); model != nullptr)
                {
                    modelVisitResponse = operator()(*model);
                }

                // If the root <model> indicated that visitation should stop, then return
                if (modelVisitResponse >= VisitModelResponse::Stop)
                {
                    return;
                }
                // Next visit any <world> tags in the SDF
                for (uint64_t worldIndex{}; worldIndex < root.WorldCount(); ++worldIndex)
                {
                    if (const sdf::World* world = root.WorldByIndex(worldIndex); world != nullptr)
                    {
                        // Delegate to the sdf::World call operator overload to visit any <model> tags in the World
                        if (VisitModelResponse worldVisitResponse = operator()(*world); worldVisitResponse >= VisitModelResponse::Stop)
                        {
                            break;
                        }
                    }
                }
            }

        public:
            ModelVisitorCallback m_modelVisitorCB;
            bool m_recurseModels{};
        };

        VisitModelsForNestedModels_fn VisitModelsForNestedModels{};
        VisitModelsForNestedModels.m_modelVisitorCB = modelVisitorCB;
        VisitModelsForNestedModels.m_recurseModels = visitNestedModels;
        VisitModelsForNestedModels(sdfRoot);
    }

    AZStd::unordered_set<AZStd::string> GetMeshesFilenames(const sdf::Root& root, bool visual, bool colliders)
    {
        AZStd::unordered_set<AZStd::string> filenames;
        auto GetMeshesFromModel = [&filenames, visual, colliders](const sdf::Model& model) -> VisitModelResponse
        {
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

            for (uint64_t index = 0; index < model.LinkCount(); index++)
            {
                processLink(model.LinkByIndex(index));
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };

        VisitModels(root, GetMeshesFromModel);

        return filenames;
    }

    const sdf::Model* GetModelContainingLink(const sdf::Root& root, AZStd::string_view linkName)
    {
        const sdf::Model* resultModel{};
        auto IsLinkInModel = [&linkName, &resultModel](const sdf::Model& model) -> VisitModelResponse
        {
            const std::string stdLinkName(linkName.data(), linkName.size());
            if (const sdf::Link* searchLink = model.LinkByName(stdLinkName); searchLink != nullptr)
            {
                resultModel = &model;
                return VisitModelResponse::Stop;
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };
        VisitModels(root, IsLinkInModel);

        return resultModel;
    }

    const sdf::Model* GetModelContainingLink(const sdf::Root& root, const sdf::Link& link)
    {
        const sdf::Model* resultModel{};
        auto IsLinkInModel = [&link, &resultModel](const sdf::Model& model) -> VisitModelResponse
        {
            if (const sdf::Link* searchLink = model.LinkByName(link.Name()); searchLink != nullptr)
            {
                resultModel = &model;
                return VisitModelResponse::Stop;
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };
        VisitModels(root, IsLinkInModel);

        return resultModel;
    }

    const sdf::Model* GetModelContainingJoint(const sdf::Root& root, AZStd::string_view jointName)
    {
        const sdf::Model* resultModel{};
        auto IsJointInModel = [&jointName, &resultModel](const sdf::Model& model) -> VisitModelResponse
        {
            const std::string stdJointName(jointName.data(), jointName.size());
            if (const sdf::Joint* searchJoint = model.JointByName(stdJointName); searchJoint != nullptr)
            {
                resultModel = &model;
                return VisitModelResponse::Stop;
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };
        VisitModels(root, IsJointInModel);

        return resultModel;
    }

    const sdf::Model* GetModelContainingJoint(const sdf::Root& root, const sdf::Joint& joint)
    {
        const sdf::Model* resultModel{};
        auto IsJointInModel = [&joint, &resultModel](const sdf::Model& model) -> VisitModelResponse
        {
            if (const sdf::Joint* searchJoint = model.JointByName(joint.Name()); searchJoint != nullptr)
            {
                resultModel = &model;
                return VisitModelResponse::Stop;
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };
        VisitModels(root, IsJointInModel);

        return resultModel;
    }

    /// Finds global path from URDF path
    AZ::IO::Path ResolveURDFPath(
        AZ::IO::Path unresolvedPath,
        const AZ::IO::PathView& urdfFilePath,
        const AZ::IO::PathView& amentPrefixPath,
        const FileExistsCB& fileExists)
    {
        AZ_Printf("ResolveURDFPath", "ResolveURDFPath with %s\n", unresolvedPath.c_str());

        // TODO: Query URDF prefix map from Settings Registry
        AZStd::vector<AZ::IO::Path> amentPrefixPaths;

        // Split the AMENT_PREFIX_PATH into multiple paths
        auto AmentPrefixPathVisitor = [&amentPrefixPaths](AZStd::string_view prefixPath)
        {
            amentPrefixPaths.push_back(prefixPath);
        };
        // Note this code only works on Unix platforms
        // For Windows this will not work as the drive letter has a colon in it (C:\)
        AZ::StringFunc::TokenizeVisitor(amentPrefixPath.Native(), AmentPrefixPathVisitor, ':');

        // Append the urdf file ancestor directories to the candidate replacement paths
        AZStd::vector<AZ::IO::Path> urdfAncestorPaths;
        if (!urdfFilePath.empty())
        {
            AZ::IO::Path urdfFileAncestorPath = urdfFilePath;
            bool rootPathVisited = false;
            do
            {
                AZ::IO::PathView parentPath = urdfFileAncestorPath.ParentPath();
                rootPathVisited = (urdfFileAncestorPath == parentPath);
                urdfAncestorPaths.emplace_back(parentPath);
                urdfFileAncestorPath = parentPath;
            } while (!rootPathVisited);
        }

        // Structure which accepts a callback that can convert an unresolved URI path(package://, model://, file://, etc...)
        // to a filesystem path
        struct UriPrefix
        {
            using SchemeResolver = AZStd::function<AZStd::optional<AZ::IO::Path>(AZ::IO::PathView)>;
            SchemeResolver m_schemeResolver;
        };

        auto GetReplacementSchemeResolver = [](AZStd::string_view schemePrefix,
                                               AZStd::span<const AZ::IO::Path> amentPrefixPaths,
                                               AZStd::span<const AZ::IO::Path> urdfAncestorPaths,
                                               const FileExistsCB& fileExistsCB)
        {
            return [schemePrefix, amentPrefixPaths, urdfAncestorPaths, &fileExistsCB](
                       AZ::IO::PathView uriPath) -> AZStd::optional<AZ::IO::Path>
            {
                // Note this is a case-sensitive check to match the exact URI scheme
                // If that is not desired, then this code should be updated to read
                // a value from the Setting Registry indicating whether the uriPrefix matching
                // should be case sensitive
                bool uriPrefixMatchCaseSensitive = true;
                // Check if the path starts with the URI scheme prefix
                if (AZ::StringFunc::StartsWith(uriPath.Native(), schemePrefix, uriPrefixMatchCaseSensitive))
                {
                    // Strip the number of characters from the Uri scheme from beginning of the path
                    AZ::IO::PathView strippedUriPath = uriPath.Native().substr(schemePrefix.size());
                    if (strippedUriPath.empty())
                    {
                        // The stripped URI path is empty, so there is nothing to resolve
                        return AZStd::nullopt;
                    }

                    // Check to see if the relative part of the URI path refers to a location
                    // within each <ament prefix path>/share directory
                    for (const AZ::IO::Path& amentPrefixPath : amentPrefixPaths)
                    {
                        auto pathIter = strippedUriPath.begin();
                        AZ::IO::PathView packageName = *pathIter;
                        const AZ::IO::Path amentSharePath = amentPrefixPath / "share";
                        const AZ::IO::Path packageManifestPath = amentSharePath / packageName / "package.xml";

                        if (const AZ::IO::Path candidateResolvedPath = amentSharePath / strippedUriPath;
                            fileExistsCB(packageManifestPath) && fileExistsCB(candidateResolvedPath))
                        {
                            return candidateResolvedPath;
                        }
                    }

                    // The URI path cannot be resolved within the any ament prefix path,
                    // so try the directory containing the URDF file as well as any of its parent directories
                    for (const AZ::IO::Path& urdfAncestorPath : urdfAncestorPaths)
                    {
                        if (const AZ::IO::Path candidateResolvedPath = urdfAncestorPath / strippedUriPath;
                            fileExistsCB(candidateResolvedPath))
                        {
                            return candidateResolvedPath;
                        }
                    }
                }

                return AZStd::nullopt;
            };
        };

        constexpr AZStd::string_view PackageSchemePrefix = "package://";
        UriPrefix packageUriPrefix;
        packageUriPrefix.m_schemeResolver =
            GetReplacementSchemeResolver(PackageSchemePrefix, amentPrefixPaths, urdfAncestorPaths, fileExists);

        constexpr AZStd::string_view ModelSchemePrefix = "model://";
        UriPrefix modelUriPrefix;
        modelUriPrefix.m_schemeResolver = GetReplacementSchemeResolver(ModelSchemePrefix, amentPrefixPaths, urdfAncestorPaths, fileExists);

        // For a local file path convert the file URI to a local path
        UriPrefix fileUriPrefix;
        fileUriPrefix.m_schemeResolver = [](AZ::IO::PathView uriPath) -> AZStd::optional<AZ::IO::Path>
        {
            constexpr AZStd::string_view FileSchemePrefix = "file://";
            // Paths that start with 'file:///' are absolute paths, so only 'file://' needs to be stripped
            bool uriPrefixMatchCaseSensitive = true;
            if (AZ::StringFunc::StartsWith(uriPath.Native(), FileSchemePrefix, uriPrefixMatchCaseSensitive))
            {
                AZStd::string_view strippedUriPath = uriPath.Native().substr(FileSchemePrefix.size());
                return AZ::IO::Path(strippedUriPath);
            }

            return AZStd::nullopt;
        };

        // Step 1: Attempt to resolved URI scheme paths
        // libsdformat seems to convert package:// references to model:// references
        // So the model:// URI prefix resolver is run first
        const auto uriPrefixes =
            AZStd::to_array<UriPrefix>({ AZStd::move(modelUriPrefix), AZStd::move(fileUriPrefix), AZStd::move(packageUriPrefix) });
        for (const UriPrefix& uriPrefix : uriPrefixes)
        {
            if (auto resolvedPath = uriPrefix.m_schemeResolver(unresolvedPath); resolvedPath.has_value())
            {
                AZ_Printf(
                    "ResolveURDFPath",
                    R"(Resolved Path using URI Prefix "%.*s" -> "%.*s")"
                    "\n",
                    AZ_PATH_ARG(unresolvedPath),
                    AZ_PATH_ARG(resolvedPath.value()));
                return resolvedPath.value();
            }
        }

        // At this point, the path has no URI scheme
        if (unresolvedPath.IsAbsolute())
        {
            AZ_Printf("ResolveURDFPath", "Input Path is an absolute local filesystem path to : %s\n", unresolvedPath.c_str());
            return unresolvedPath;
        }

        // The path is relative path, so append to the directory containing the .urdf file
        const AZ::IO::Path resolvedPath = AZ::IO::Path(urdfFilePath.ParentPath()) / unresolvedPath;
        AZ_Printf("ResolveURDFPath", "Input Path %s is being returned as is\n", unresolvedPath.c_str());
        return resolvedPath;
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
