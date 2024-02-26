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
#include <AzCore/Utils/Utils.h>
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

    AZ::Transform GetLocalTransformURDF(const sdf::SemanticPose& semanticPose, AZ::Transform t)
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

        const AZ::Transform localTransform = URDF::TypeConversions::ConvertPose(resolvedPose);
        const AZ::Transform resolvedTransform = localTransform * t;
        return resolvedTransform;
    }

    void VisitLinks(const sdf::Model& sdfModel, const LinkVisitorCallback& linkVisitorCB, bool visitNestedModelLinks)
    {
        // Function object which can visit all links of a model
        // Optionally it supports recursing nested models to visit their links as well
        struct VisitLinksForNestedModels_fn
        {
            VisitModelResponse operator()(const sdf::Model& model)
            {
                m_modelStack.push_back(model);
                VisitModelResponse visitResponse = VisitLinksForModel(model);
                if (m_recurseModels && visitResponse == VisitModelResponse::VisitNestedAndSiblings)
                {
                    // Nested model link are only visited if the joint visitor returns true
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        if (const sdf::Model* nestedModel = model.ModelByIndex(modelIndex); nestedModel != nullptr)
                        {
                            if (VisitModelResponse nestedVisitResponse = operator()(*nestedModel);
                                nestedVisitResponse >= VisitModelResponse::Stop)
                            {
                                // Sibling nested model links are only visited
                                // if the joint visitor returns true
                                break;
                            }
                        }
                    }
                }
                m_modelStack.pop_back();

                return visitResponse;
            }

        private:
            //! Returns success by default
            //! But an invoked visitor can return false to halt further iteration
            VisitModelResponse VisitLinksForModel(const sdf::Model& currentModel)
            {
                for (uint64_t linkIndex{}; linkIndex < currentModel.LinkCount(); ++linkIndex)
                {
                    if (const sdf::Link* link = currentModel.LinkByIndex(linkIndex); link != nullptr)
                    {
                        if (!m_linkVisitorCB(*link, m_modelStack))
                        {
                            return VisitModelResponse::Stop;
                        }
                    }
                }

                return VisitModelResponse::VisitNestedAndSiblings;
            }

        public:
            LinkVisitorCallback m_linkVisitorCB;
            bool m_recurseModels{};

        private:
            // Stack storing the current composition of models visited so far
            ModelStack m_modelStack;
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
            VisitModelResponse operator()(const sdf::Model& model)
            {
                m_modelStack.push_back(model);
                VisitModelResponse visitResponse = VisitJointsForModel(model);
                if (m_recurseModels && visitResponse == VisitModelResponse::VisitNestedAndSiblings)
                {
                    // Nested model joints are only visited if the joint visitor returns true
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        if (const sdf::Model* nestedModel = model.ModelByIndex(modelIndex); nestedModel != nullptr)
                        {
                            if (VisitModelResponse nestedVisitResponse = operator()(*nestedModel);
                                nestedVisitResponse >= VisitModelResponse::Stop)
                            {
                                // Sibling nested model joints are only visited
                                // if the joint visitor returns true
                                break;
                            }
                        }
                    }
                }
                m_modelStack.pop_back();

                return visitResponse;
            }

        private:
            VisitModelResponse VisitJointsForModel(const sdf::Model& currentModel)
            {
                for (uint64_t jointIndex{}; jointIndex < currentModel.JointCount(); ++jointIndex)
                {
                    const sdf::Joint* joint = currentModel.JointByIndex(jointIndex);
                    // Skip any joints whose parent and child link references
                    // don't have an actual sdf::link in the parsed model
                    if (joint != nullptr && currentModel.LinkNameExists(joint->ParentName()) && currentModel.LinkByName(joint->ChildName()))
                    {
                        if (!m_jointVisitorCB(*joint, m_modelStack))
                        {
                            return VisitModelResponse::Stop;
                        }
                    }
                }

                return VisitModelResponse::VisitNestedAndSiblings;
            }

        public:
            JointVisitorCallback m_jointVisitorCB;
            bool m_recurseModels{};

        private:
            // Stack storing the current composition of models visited so far
            ModelStack m_modelStack;
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

    //! Provides overloads for comparison operators for the VisitModelResponse enum
    AZ_DEFINE_ENUM_RELATIONAL_OPERATORS(VisitModelResponse);

    // Function object which can visit all models in an SDF document
    // Optionally it supports recursing nested models as well
    struct VisitModelsForNestedModels_fn
    {
        VisitModelResponse operator()(const sdf::Model& model)
        {
            // The VisitModelResponse enum value is used to filter out
            // less callbacks the higher the value grows.
            // So any values above VisitNestedAndSiblings will not visit nested models
            VisitModelResponse visitResponse = m_modelVisitorCB(model, m_modelStack);

            if (m_recurseModels && visitResponse == VisitModelResponse::VisitNestedAndSiblings)
            {
                // Nested models are only visited if the model visitor returns VisitNestedAndSiblings
                m_modelStack.push_back(model);
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
                m_modelStack.pop_back();
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
                    // Stop visiting the world's <model> children if any children return Stop
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
            // Visit all <model> tags at the root of the SDF
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

    private:
        // Stack storing the current composition of models visited so far
        ModelStack m_modelStack;
    };

    void VisitModels(const sdf::Root& sdfRoot, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels)
    {
        VisitModelsForNestedModels_fn VisitModelsForNestedModels{};
        VisitModelsForNestedModels.m_modelVisitorCB = modelVisitorCB;
        VisitModelsForNestedModels.m_recurseModels = visitNestedModels;
        VisitModelsForNestedModels(sdfRoot);
    }

    void VisitModels(const sdf::World& sdfWorld, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels)
    {
        VisitModelsForNestedModels_fn VisitModelsForNestedModels{};
        VisitModelsForNestedModels.m_modelVisitorCB = modelVisitorCB;
        VisitModelsForNestedModels.m_recurseModels = visitNestedModels;
        VisitModelsForNestedModels(sdfWorld);
    }

    void VisitModels(const sdf::Model& sdfModel, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels)
    {
        VisitModelsForNestedModels_fn VisitModelsForNestedModels{};
        VisitModelsForNestedModels.m_modelVisitorCB = modelVisitorCB;
        VisitModelsForNestedModels.m_recurseModels = visitNestedModels;
        VisitModelsForNestedModels(sdfModel);
    }

    ModelMap GetAllModels(const sdf::Root& sdfRoot, bool gatherNestedModelsForModel)
    {
        ModelMap modelMap;
        auto GatherModels = [&modelMap](const sdf::Model& nestedModel, const ModelStack& modelStack) -> VisitModelResponse
        {
            std::string fullyQualifiedModelName;
            // Prepend the Model names to the joint name using the Name Scoping support in libsdformat
            // http://sdformat.org/tutorials?tut=composition_proposal#1-3-name-scoping-and-cross-referencing
            for (const sdf::Model& model : modelStack)
            {
                fullyQualifiedModelName = sdf::JoinName(fullyQualifiedModelName, model.Name());
            }
            fullyQualifiedModelName = sdf::JoinName(fullyQualifiedModelName, nestedModel.Name());

            AZStd::string azFullModelName(fullyQualifiedModelName.c_str(), fullyQualifiedModelName.size());
            modelMap.insert_or_assign(AZStd::move(azFullModelName), &nestedModel);
            return VisitModelResponse::VisitNestedAndSiblings;
        };

        VisitModels(sdfRoot, GatherModels, gatherNestedModelsForModel);
        return modelMap;
    }

    ModelMap GetAllModels(const sdf::World& sdfWorld, bool gatherNestedModelsForModel)
    {
        ModelMap modelMap;
        auto GatherModels = [&modelMap](const sdf::Model& nestedModel, const ModelStack& modelStack) -> VisitModelResponse
        {
            std::string fullyQualifiedModelName;
            // Prepend the Model names to the joint name using the Name Scoping support in libsdformat
            // http://sdformat.org/tutorials?tut=composition_proposal#1-3-name-scoping-and-cross-referencing
            for (const sdf::Model& model : modelStack)
            {
                fullyQualifiedModelName = sdf::JoinName(fullyQualifiedModelName, model.Name());
            }
            fullyQualifiedModelName = sdf::JoinName(fullyQualifiedModelName, nestedModel.Name());

            AZStd::string azFullModelName(fullyQualifiedModelName.c_str(), fullyQualifiedModelName.size());
            modelMap.insert_or_assign(AZStd::move(azFullModelName), &nestedModel);
            return VisitModelResponse::VisitNestedAndSiblings;
        };

        VisitModels(sdfWorld, GatherModels, gatherNestedModelsForModel);
        return modelMap;
    }

    ModelMap GetAllModels(const sdf::Model& sdfModel, bool gatherNestedModelsForModel)
    {
        ModelMap modelMap;
        auto GatherModels = [&modelMap](const sdf::Model& nestedModel, const ModelStack& modelStack) -> VisitModelResponse
        {
            std::string fullyQualifiedModelName;
            // Prepend the Model names to the joint name using the Name Scoping support in libsdformat
            // http://sdformat.org/tutorials?tut=composition_proposal#1-3-name-scoping-and-cross-referencing
            for (const sdf::Model& model : modelStack)
            {
                fullyQualifiedModelName = sdf::JoinName(fullyQualifiedModelName, model.Name());
            }
            fullyQualifiedModelName = sdf::JoinName(fullyQualifiedModelName, nestedModel.Name());

            AZStd::string azFullModelName(fullyQualifiedModelName.c_str(), fullyQualifiedModelName.size());
            modelMap.insert_or_assign(AZStd::move(azFullModelName), &nestedModel);
            return VisitModelResponse::VisitNestedAndSiblings;
        };

        VisitModels(sdfModel, GatherModels, gatherNestedModelsForModel);
        return modelMap;
    }

    const sdf::Model* GetModelContainingLink(const sdf::Root& root, AZStd::string_view fullyQualifiedLinkName)
    {
        const sdf::Model* resultModel{};
        auto IsLinkInModel = [&fullyQualifiedLinkName,
                              &resultModel](const sdf::Model& model, const ModelStack&) -> VisitModelResponse
        {
            const std::string stdLinkName(fullyQualifiedLinkName.data(), fullyQualifiedLinkName.size());
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
        auto IsLinkInModel = [&link, &resultModel](const sdf::Model& model, const ModelStack&) -> VisitModelResponse
        {
            if (const sdf::Link* searchLink = model.LinkByName(link.Name()); searchLink != &link)
            {
                resultModel = &model;
                return VisitModelResponse::Stop;
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };
        VisitModels(root, IsLinkInModel);

        return resultModel;
    }

    const sdf::Model* GetModelContainingJoint(const sdf::Root& root, AZStd::string_view fullyQualifiedJointName)
    {
        const sdf::Model* resultModel{};
        auto IsJointInModel = [&fullyQualifiedJointName, &resultModel](const sdf::Model& model, const ModelStack&) -> VisitModelResponse
        {
            const std::string stdJointName(fullyQualifiedJointName.data(), fullyQualifiedJointName.size());
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
        auto IsJointInModel = [&joint, &resultModel](const sdf::Model& model, const ModelStack&) -> VisitModelResponse
        {
            if (const sdf::Joint* searchJoint = model.JointByName(joint.Name()); searchJoint != &joint)
            {
                resultModel = &model;
                return VisitModelResponse::Stop;
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };
        VisitModels(root, IsJointInModel);

        return resultModel;
    }

    const sdf::Model* GetModelContainingModel(const sdf::Root& root, const sdf::Model& model)
    {
        const sdf::Model* resultModel{};
        auto IsModelInModel = [&model, &resultModel](const sdf::Model& outerModel, const ModelStack&) -> VisitModelResponse
        {
            // Validate the memory address of the model matches the outer model found searching the visited model "child models"
            if (const sdf::Model* searchModel = outerModel.ModelByName(model.Name()); searchModel != &model)
            {
                resultModel = &outerModel;
                return VisitModelResponse::Stop;
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };
        VisitModels(root, IsModelInModel);

        return resultModel;
    }

    AssetFilenameReferences GetReferencedAssetFilenames(const sdf::Root& root)
    {
        AssetFilenameReferences filenames;
        auto GetAssetsFromModel = [&filenames](const sdf::Model& model, const ModelStack&) -> VisitModelResponse
        {
            const auto addFilenameFromGeometry = [&filenames](const sdf::Geometry* geometry, ReferencedAssetType assetType)
            {
                if (geometry->Type() == sdf::GeometryType::MESH)
                {
                    if (auto mesh = geometry->MeshShape(); mesh)
                    {
                        AZStd::string uri(mesh->Uri().c_str(), mesh->Uri().size());
                        if (filenames.contains(uri))
                        {
                            filenames[uri] = filenames[uri] | assetType;
                        }
                        else
                        {
                            filenames.emplace(uri, assetType);
                        }
                    }
                }
            };

            const auto addFilenamesFromMaterial = [&filenames](const sdf::Material* material)
            {
                // Only PBR entries on a material have filenames that need to be added.
                if ((!material) || (!material->PbrMaterial()))
                {
                    return;
                }

                if (auto pbr = material->PbrMaterial(); pbr)
                {
                    auto pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::METAL);
                    if (!pbrWorkflow)
                    {
                        pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
                        if (!pbrWorkflow)
                        {
                            return;
                        }
                    }

                    if (auto texture = pbrWorkflow->AlbedoMap(); !texture.empty())
                    {
                        filenames.emplace(AZStd::string(texture.c_str(), texture.size()), ReferencedAssetType::Texture);
                    }

                    if (auto texture = pbrWorkflow->NormalMap(); !texture.empty())
                    {
                        filenames.emplace(AZStd::string(texture.c_str(), texture.size()), ReferencedAssetType::Texture);
                    }

                    if (auto texture = pbrWorkflow->AmbientOcclusionMap(); !texture.empty())
                    {
                        filenames.emplace(AZStd::string(texture.c_str(), texture.size()), ReferencedAssetType::Texture);
                    }

                    if (auto texture = pbrWorkflow->EmissiveMap(); !texture.empty())
                    {
                        filenames.emplace(AZStd::string(texture.c_str(), texture.size()), ReferencedAssetType::Texture);
                    }

                    if (pbrWorkflow->Type() == sdf::PbrWorkflowType::METAL)
                    {
                        if (auto texture = pbrWorkflow->RoughnessMap(); !texture.empty())
                        {
                            filenames.emplace(AZStd::string(texture.c_str(), texture.size()), ReferencedAssetType::Texture);
                        }

                        if (auto texture = pbrWorkflow->MetalnessMap(); !texture.empty())
                        {
                            filenames.emplace(AZStd::string(texture.c_str(), texture.size()), ReferencedAssetType::Texture);
                        }
                    }
                }
            };

            const auto processLink = [&addFilenameFromGeometry, &addFilenamesFromMaterial](const sdf::Link* link)
            {
                for (uint64_t index = 0; index < link->VisualCount(); index++)
                {
                    addFilenameFromGeometry(link->VisualByIndex(index)->Geom(), ReferencedAssetType::VisualMesh);
                    addFilenamesFromMaterial(link->VisualByIndex(index)->Material());
                }

                for (uint64_t index = 0; index < link->CollisionCount(); index++)
                {
                    addFilenameFromGeometry(link->CollisionByIndex(index)->Geom(), ReferencedAssetType::ColliderMesh);
                }
            };

            for (uint64_t index = 0; index < model.LinkCount(); index++)
            {
                processLink(model.LinkByIndex(index));
            }

            return VisitModelResponse::VisitNestedAndSiblings;
        };

        VisitModels(root, GetAssetsFromModel);

        return filenames;
    }

    AZ::IO::Path ResolveAmentPrefixPath(
        AZ::IO::Path unresolvedPath,
        AZStd::string_view amentPrefixPath,
        const FileExistsCB& fileExistsCB)
    {
        AZStd::vector<AZ::IO::Path> amentPrefixPaths;

        // Parse the AMENT_PREFIX_PATH environment variable into a set of distinct paths.
        auto AmentPrefixPathVisitor = [&amentPrefixPaths](
            AZStd::string_view prefixPath)
        {
            amentPrefixPaths.push_back(prefixPath);
        };
        // Note this code only works on Unix platforms
        // For Windows this will not work as the drive letter has a colon in it (C:\)
        AZ::StringFunc::TokenizeVisitor(amentPrefixPath, AmentPrefixPathVisitor, ':');

        AZ::IO::PathView strippedPath;

        // The AMENT_PREFIX_PATH is only used for lookups if the URI starts with "model://" or "package://"
        constexpr AZStd::string_view ValidAmentPrefixes[] = {"model://", "package://"};
        for (const auto& prefix : ValidAmentPrefixes)
        {
            // Perform a case-sensitive check to look for the prefix.
            constexpr bool prefixMatchCaseSensitive = true;
            if (AZ::StringFunc::StartsWith(unresolvedPath.Native(), prefix, prefixMatchCaseSensitive))
            {
                strippedPath = AZ::IO::PathView(unresolvedPath).Native().substr(prefix.size());
                break;
            }
        }

        // If no valid prefix was found, or if the URI *only* contains the prefix, return an empty result.
        if (strippedPath.empty())
        {
            return {};
        }

        // If the remaining path is an absolute path, it shouldn't get resolved with AMENT_PREFIX_PATH. return an empty result.
        if (strippedPath.IsAbsolute())
        {
            return {};
        }

        // Check to see if the relative part of the URI path refers to a location
        // within each <ament prefix path>/share directory
        for (const AZ::IO::Path& amentPrefixPath : amentPrefixPaths)
        {
            auto pathIter = strippedPath.begin();
            AZ::IO::PathView packageName = *pathIter;
            const AZ::IO::Path amentSharePath = amentPrefixPath / "share";
            const AZ::IO::Path packageManifestPath = amentSharePath / packageName / "package.xml";

            // Given a path like 'ambulance/meshes/model.stl', it will be considered a match if
            // <ament prefix path>/share/ambulance/package.xml exists and
            // <ament prefix path>/share/ambulance/meshes/model.stl exists.
            if (const AZ::IO::Path candidateResolvedPath = amentSharePath / strippedPath;
                fileExistsCB(packageManifestPath) && fileExistsCB(candidateResolvedPath))
            {
                AZ_Trace("ResolveAssetPath", R"(Resolved using AMENT_PREFIX_PATH: "%.*s" -> "%.*s")" "\n",
                    AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(candidateResolvedPath));
                return candidateResolvedPath;
            }
        }

        // No resolution was found, return an empty result.
        return {};
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

        // If the settings tell us to try the AMENT_PREFIX_PATH, use that first to try and resolve path.
        if (pathResolverSettings.m_useAmentPrefixPath)
        {
            if (AZ::IO::Path amentResolvedPath = ResolveAmentPrefixPath(unresolvedPath, amentPrefixPath, fileExistsCB); !amentResolvedPath.empty())
            {
                return amentResolvedPath;
            }
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

        // Loop through each prefix in the builder settings and attempt to resolve it as either an absolute path
        // or a relative path that's relative in some way to the base file.
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

            // Loop through each replacement path for this prefix, attach it to the front, and look for matches.
            for (const auto& replacement : replacements)
            {
                AZ::IO::Path replacedUriPath(replacement);
                replacedUriPath /= strippedUriPath;

                // If we successfully matched the prefix, and the replacement path is completely empty, we don't need to look any further.
                // There's no match.
                if (replacedUriPath.empty())
                {
                    AZ_Trace("ResolveAssetPath", R"(Resolved Path is empty: "%.*s" -> "")" "\n", AZ_PATH_ARG(unresolvedPath));
                    return {};
                }

                // If the replaced path is an absolute path, if it exists, return it.
                // If it doesn't exist, keep trying other replacements.
                if (replacedUriPath.IsAbsolute())
                {
                    if (fileExistsCB(replacedUriPath))
                    {
                        AZ_Trace("ResolveAssetPath", R"(Resolved Absolute Path: "%.*s" -> "%.*s")" "\n",
                            AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(replacedUriPath));
                        return replacedUriPath;
                    }
                    else
                    {
                        // The file didn't exist, so continue.
                        continue;
                    }
                }

                // The URI path is not absolute, so attempt to append it to the ancestor directories of the URDF/SDF file
                for (const AZ::IO::Path& ancestorPath : ancestorPaths)
                {
                    if (const AZ::IO::Path candidateResolvedPath = ancestorPath / replacedUriPath;
                        fileExistsCB(candidateResolvedPath))
                    {
                        AZ_Trace("ResolveAssetPath", R"(Resolved using ancestor paths: "%.*s" -> "%.*s")" "\n",
                            AZ_PATH_ARG(unresolvedPath), AZ_PATH_ARG(candidateResolvedPath));
                        return candidateResolvedPath;
                    }
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
                return {};
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
        return {};
    }
    AmentPrefixString GetAmentPrefixPath()
    {
        // Support reading the AMENT_PREFIX_PATH environment variable on Unix/Windows platforms
        auto StoreAmentPrefixPath = [](char* buffer, size_t size) -> size_t
        {
            auto getEnvOutcome = AZ::Utils::GetEnv(AZStd::span(buffer, size), "AMENT_PREFIX_PATH");

            if (!getEnvOutcome.IsSuccess())
            {
                if (getEnvOutcome.GetError().m_errorCode == AZ::Utils::GetEnvErrorCode::EnvNotSet)
                {
                    AZ_Error("UrdfAssetMap", false, "AMENT_PREFIX_PATH is not set in the environment.");
                }
                else if (getEnvOutcome.GetError().m_errorCode == AZ::Utils::GetEnvErrorCode::BufferTooSmall)
                {
                    AZ_Error(
                        "UrdfAssetMap",
                        false,
                        "AMENT_PREFIX_PATH is too long (%zu), maximum permissible size is %zu ",
                        getEnvOutcome.GetError().m_requiredSize,
                        size);
                }
                else
                {
                    AZ_Error("UrdfAssetMap", false, "AMENT_PREFIX_PATH is not found.");
                }
            }

            return getEnvOutcome ? getEnvOutcome.GetValue().size() : 0;
        };
        AmentPrefixString amentPrefixPath;
        amentPrefixPath.resize_and_overwrite(amentPrefixPath.capacity(), StoreAmentPrefixPath);

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

    sdf::ParserConfig CreateSdfParserConfigFromSettings(const SdfAssetBuilderSettings& settings, const AZ::IO::PathView& baseFilePath)
    {
        sdf::ParserConfig sdfConfig;

        sdfConfig.URDFSetPreserveFixedJoint(settings.m_urdfPreserveFixedJoints);

        // Fill in the URI resolution with the supplied prefix mappings.
        for (auto& [prefix, pathList] : settings.m_resolverSettings.m_uriPrefixMap)
        {
            std::string uriPath;
            for (auto& path : pathList)
            {
                if (!uriPath.empty())
                {
                    uriPath.append(std::string(":"));
                }

                uriPath.append(std::string(path.c_str(), path.size()));
            }
            if (!prefix.empty() && !uriPath.empty())
            {
                std::string uriPrefix(prefix.c_str(), prefix.size());
                sdfConfig.AddURIPath(uriPrefix, uriPath);
                AZ_Trace("SdfParserConfig", "Added URI mapping '%s' -> '%s'", uriPrefix.c_str(), uriPath.c_str());
            }
        }

        // If any files couldn't be found using our supplied prefix mappings, this callback will get called.
        // Attempt to use our full path resolution, and print a warning if it still couldn't be resolved.
        sdfConfig.SetFindCallback([settings, baseFilePath](const std::string &fileName) -> std::string
        {
            auto amentPrefixPath = Utils::GetAmentPrefixPath();

            auto resolved = Utils::ResolveAssetPath(AZ::IO::Path(fileName.c_str()), baseFilePath, amentPrefixPath, settings);
            if (!resolved.empty())
            {
                AZ_Trace("SdfParserConfig", "SDF SetFindCallback resolved '%s' -> '%s'", fileName.c_str(), resolved.c_str());
                return resolved.c_str();
            }

            AZ_Warning("SdfParserConfig", false, "SDF SetFindCallback failed to resolve '%s'", fileName.c_str());
            return fileName;
        });

        return sdfConfig;
    }
} // namespace ROS2::Utils::SDFormat
