/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "URDFPrefabMaker.h"
#include "CollidersMaker.h"
#include "PrefabMakerUtils.h"
#include <API/EditorAssetSystemAPI.h>
#include <Atom/Feature/Mesh/MeshFeatureProcessor.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzFramework/Scene/SceneSystemInterface.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabLoaderScriptingBus.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponentInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemScriptingBus.h>
#include <AzToolsFramework/Prefab/Procedural/ProceduralPrefabAsset.h>
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/Utils/TypeConversions.h>

namespace ROS2
{
    URDFPrefabMaker::URDFPrefabMaker(
        const AZStd::string& modelFilePath,
        const sdf::Root* root,
        AZStd::string prefabPath,
        const AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetsMapping,
        bool useArticulations,
        const AZStd::optional<AZ::Transform> spawnPosition)
        : m_root(root)
        , m_visualsMaker(urdfAssetsMapping)
        , m_collidersMaker(urdfAssetsMapping)
        , m_prefabPath(std::move(prefabPath))
        , m_urdfAssetsMapping(urdfAssetsMapping)
        , m_spawnPosition(spawnPosition)
        , m_useArticulations(useArticulations)
    {
        AZ_Assert(!m_prefabPath.empty(), "Prefab path is empty");
        AZ_Assert(m_root, "SDF Root is nullptr");
    }

    URDFPrefabMaker::CreatePrefabTemplateResult URDFPrefabMaker::CreatePrefabTemplateFromUrdfOrSdf()
    {
        {
            AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
            m_status.clear();
            m_articulationsCounter = 0u;
        }

        if (!ContainsModel())
        {
            return AZ::Failure(AZStd::string("URDF/SDF doesn't contain any models."));
        }

        // Visit any nested models in the SDF as well
        constexpr bool visitNestedModels = true;

        // Maintains references to all joints and a mapping from the joints to their parent and child links
        struct JointsMapper
        {
            struct JointToAttachedModel
            {
                AZStd::string m_fullyQualifiedName;
                const sdf::Joint* m_joint{};
                const sdf::Model* m_attachedModel{};
            };
            // this is a unique ordered vector
            AZStd::vector<JointToAttachedModel> m_joints;
            AZStd::unordered_map<const sdf::Joint*, const sdf::Link*> m_jointToParentLinks;
            AZStd::unordered_map<const sdf::Joint*, const sdf::Link*> m_jointToChildLinks;
        };
        JointsMapper jointsMapper;
        auto GetAllJointsFromModel = [&jointsMapper](const sdf::Model& model, const Utils::ModelStack&) -> Utils::VisitModelResponse
        {
            // As the VisitModels function visits nested models by default, gatherNestedModelJoints is set to false
            constexpr bool gatherNestedModelJoints = false;
            auto jointsForModel = Utils::GetAllJoints(model, gatherNestedModelJoints);
            for (const auto& [fullyQualifiedName, joint] : jointsForModel)
            {
                JointsMapper::JointToAttachedModel jointToAttachedModel{
                    AZStd::string(fullyQualifiedName.c_str(), fullyQualifiedName.size()), joint, &model
                };
                jointsMapper.m_joints.push_back(AZStd::move(jointToAttachedModel));

                // add mapping from joint to child link
                std::string childName = joint->ChildName();
                if (const sdf::Link* link = model.LinkByName(childName); link != nullptr)
                {
                    // Add a mapping of joint to child link
                    jointsMapper.m_jointToChildLinks[joint] = link;
                }

                // add mapping from joint to parent link
                std::string parentName = joint->ParentName();
                if (const sdf::Link* link = model.LinkByName(parentName); link != nullptr)
                {
                    jointsMapper.m_jointToParentLinks[joint] = link;
                }
            }

            return Utils::VisitModelResponse::VisitNestedAndSiblings;
        };
        // Gather all Joints in SDF including in nested models
        Utils::VisitModels(*m_root, GetAllJointsFromModel, visitNestedModels);

        // Maintains references to all Links in the SDF and a mapping of links to the model it is attached to
        struct LinksMapper
        {
            struct LinkToAttachedModel
            {
                AZStd::string m_fullyQualifiedName;
                const sdf::Link* m_link{};
                const sdf::Model* m_attachedModel{};
            };
            // this is a unique ordered vector
            AZStd::vector<LinkToAttachedModel> m_links;
        };
        LinksMapper linksMapper;

        struct ModelMapper
        {
            struct NestedModelToAttachedModel
            {
                AZStd::string m_fullyQualifiedName;
                const sdf::Model* m_nestedModel{};
                const sdf::Model* m_attachedModel{};
            };
            // this is a unique ordered vector
            AZStd::vector<NestedModelToAttachedModel> m_models;
        };
        ModelMapper modelMapper;

        auto GetAllLinksAndSetModelHierarchy =
            [&linksMapper, &modelMapper](const sdf::Model& model, const Utils::ModelStack& modelStack) -> Utils::VisitModelResponse
        {
            // As the VisitModels function visits nested models by default, gatherNestedModelLinks is set to false
            constexpr bool gatherNestedModelLinks = false;
            auto linksForModel = Utils::GetAllLinks(model, gatherNestedModelLinks);
            for (const auto& [fullyQualifiedName, link] : linksForModel)
            {
                // Push back the mapping of link to attached model into the ordered vector
                AZStd::string fullLinkName(fullyQualifiedName.c_str(), fullyQualifiedName.size());
                LinksMapper::LinkToAttachedModel linkToAttachedModel{ AZStd::move(fullLinkName), link, &model };
                linksMapper.m_links.push_back(AZStd::move(linkToAttachedModel));
            }

            // Use the model stack to create a mapping from the current model
            // to the parent model it is attached to.
            // If the current model has no parent model the attached model is set to nullptr
            std::string stdFullModelName;
            for (const sdf::Model& ancestorModel : modelStack)
            {
                stdFullModelName = sdf::JoinName(stdFullModelName, ancestorModel.Name());
            }
            stdFullModelName = sdf::JoinName(stdFullModelName, model.Name());
            AZStd::string fullModelName(stdFullModelName.c_str(), stdFullModelName.size());
            ModelMapper::NestedModelToAttachedModel nestedModelToAttachedModel;
            nestedModelToAttachedModel.m_fullyQualifiedName = AZStd::move(fullModelName);
            nestedModelToAttachedModel.m_nestedModel = &model;
            nestedModelToAttachedModel.m_attachedModel = !modelStack.empty() ? &modelStack.back().get() : nullptr;
            modelMapper.m_models.push_back(AZStd::move(nestedModelToAttachedModel));

            return Utils::VisitModelResponse::VisitNestedAndSiblings;
        };

        // Gather all links and add a mapping of nested model -> parent model for each model in the SDF
        Utils::VisitModels(*m_root, GetAllLinksAndSetModelHierarchy, visitNestedModels);

        // Build up a list of all entities created as a part of processing the file.
        AZStd::vector<AZ::EntityId> createdEntities;
        AZStd::unordered_map<const sdf::Model*, AzToolsFramework::Prefab::PrefabEntityResult> createdModels;

        AZStd::unordered_map<const sdf::Link*, AzToolsFramework::Prefab::PrefabEntityResult> createdLinks;
        AZStd::unordered_map<AZStd::string, const sdf::Link*> links;

        // Create an entity for each model
        for ([[maybe_unused]] const auto& [fullModelName, modelPtr, _] : modelMapper.m_models)
        {
            // Create entities for each model in the SDF
            const std::string modelName = modelPtr->Name();
            const AZStd::string azModelName(modelName.c_str(), modelName.size());
            if (AzToolsFramework::Prefab::PrefabEntityResult createModelEntityResult = CreateEntityForModel(*modelPtr);
                createModelEntityResult)
            {
                AZ::EntityId createdModelEntityId = createModelEntityResult.GetValue();
                // Add the model entity to the created entity list so that it gets added to the prefab
                createdEntities.emplace_back(createdModelEntityId);
                createdModels.emplace(modelPtr, createModelEntityResult);

                AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
                m_status.emplace(
                    StatusMessageType::Model,
                    AZStd::string::format("%s created as: %s", azModelName.c_str(), createdModelEntityId.ToString().c_str()));
            }
            else
            {
                AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
                m_status.emplace(
                    StatusMessageType::Model,
                    AZStd::string::format("%s failed: %s", azModelName.c_str(), createModelEntityResult.GetError().c_str()));
            }
        }

        //! Setup the parent hierarchy for the nested models
        for ([[maybe_unused]] const auto& [_, modelPtr, parentModelPtr] : modelMapper.m_models)
        {
            // If there is no parent model, then the model would be at the top level of the hierarchy
            if (parentModelPtr == nullptr || modelPtr == nullptr)
            {
                continue;
            }
            // Create entities for each model in the SDF
            AZ::EntityId modelEntityId;
            if (auto modelIt = createdModels.find(modelPtr); modelIt != createdModels.end() && modelIt->second)
            {
                modelEntityId = modelIt->second.GetValue();
            }

            AZ::EntityId parentModelEntityId;
            if (auto parentModelIt = createdModels.find(parentModelPtr); parentModelIt != createdModels.end() && parentModelIt->second)
            {
                parentModelEntityId = parentModelIt->second.GetValue();
            }

            // If the both the parent model and current model entity exist
            // set the current model transform component parent to the parent model
            if (parentModelEntityId.IsValid() && modelEntityId.IsValid())
            {
                // The model entity local transform should be used
                // to allow it to move, rotate, scale relative to the parent
                PrefabMakerUtils::SetEntityParentRelative(modelEntityId, parentModelEntityId);
            }
        }

        // Create an entity for each link and set the parent to be the model entity where the link is attached
        for ([[maybe_unused]] const auto& [_, linkPtr, attachedModel] : linksMapper.m_links)
        {
            AZ::EntityId modelEntityId;
            if (attachedModel != nullptr)
            {
                if (auto modelIt = createdModels.find(attachedModel); modelIt != createdModels.end())
                {
                    if (modelIt->second.IsSuccess())
                    {
                        modelEntityId = modelIt->second.GetValue();
                    }
                }
            }
            // Add all link as children of their attached model entity by default
            createdLinks[linkPtr] = AddEntitiesForLink(*linkPtr, attachedModel, modelEntityId, createdEntities);
        }

        for (const auto& [linkPtr, result] : createdLinks)
        {
            std::string linkName = linkPtr->Name();
            AZStd::string azLinkName(linkName.c_str(), linkName.size());
            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf",
                "Link with name %s was created as: %s\n",
                linkName.c_str(),
                result.IsSuccess() ? (result.GetValue().ToString().c_str()) : ("[Failed]"));
            AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
            if (result.IsSuccess())
            {
                m_status.emplace(
                    StatusMessageType::Link,
                    AZStd::string::format("%s created as: %s", azLinkName.c_str(), result.GetValue().ToString().c_str()));
            }
            else
            {
                m_status.emplace(
                    StatusMessageType::Link, AZStd::string::format("%s failed : %s", azLinkName.c_str(), result.GetError().c_str()));
            }
        }

        // Set the transforms of links
        for ([[maybe_unused]] const auto& [fullLinkName, linkPtr, _] : linksMapper.m_links)
        {
            if (const auto createLinkEntityResult = createdLinks.at(linkPtr); createLinkEntityResult.IsSuccess())
            {
                AZ::EntityId createdEntityId = createLinkEntityResult.GetValue();
                std::string linkName = linkPtr->Name();
                const auto linkSemanticPose = linkPtr->SemanticPose();
                AZ::Transform tf = Utils::GetLocalTransformURDF(linkSemanticPose);
                auto* entity = AzToolsFramework::GetEntityById(createdEntityId);
                if (entity)
                {
                    auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
                    if (transformInterface)
                    {
                        AZ_Trace(
                            "CreatePrefabFromUrdfOrSdf",
                            "Setting transform %s %s to [%f %f %f] [%f %f %f %f]\n",
                            linkName.c_str(),
                            createdEntityId.ToString().c_str(),
                            tf.GetTranslation().GetX(),
                            tf.GetTranslation().GetY(),
                            tf.GetTranslation().GetZ(),
                            tf.GetRotation().GetX(),
                            tf.GetRotation().GetY(),
                            tf.GetRotation().GetZ(),
                            tf.GetRotation().GetW());
                        transformInterface->SetLocalTM(tf);
                    }
                    else
                    {
                        AZ_Trace(
                            "CreatePrefabFromUrdfOrSdf",
                            "Setting transform failed: %s does not have transform interface\n",
                            linkName.c_str());
                    }
                }
            }
        }

        // Set the hierarchy
        AZStd::vector<AZStd::pair<AZ::EntityId, const sdf::Model*>> linkEntityIdsWithoutParent;
        for (const auto& [fullLinkName, linkPtr, attachedModel] : linksMapper.m_links)
        {
            std::string linkName = linkPtr->Name();
            AZStd::string azLinkName(linkName.c_str(), linkName.size());
            const auto linkPrefabResult = createdLinks.at(linkPtr);
            if (!linkPrefabResult.IsSuccess())
            {
                AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s creation failed\n", fullLinkName.c_str());
                continue;
            }

            AZStd::vector<const sdf::Joint*> jointsWhereLinkIsChild;
            bool gatherNestedModelJoints = true;
            jointsWhereLinkIsChild = Utils::GetJointsForChildLink(*attachedModel, azLinkName, gatherNestedModelJoints);

            if (jointsWhereLinkIsChild.empty())
            {
                // emplace unique entry to the container of links that don't have a parent link associated with it
                auto linkPrefabTest = [&linkPrefabResult](const AZStd::pair<AZ::EntityId, const sdf::Model*>& query)
                {
                    return (query.first == linkPrefabResult.GetValue());
                };
                if (auto existingLinkIt =
                        AZStd::find_if(linkEntityIdsWithoutParent.begin(), linkEntityIdsWithoutParent.end(), linkPrefabTest);
                    existingLinkIt == linkEntityIdsWithoutParent.end())
                {
                    linkEntityIdsWithoutParent.emplace_back(AZStd::make_pair(linkPrefabResult.GetValue(), attachedModel));
                }
                AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s has no parents\n", linkName.c_str());
                continue;
            }

            // For URDF, a link can only be child in a single joint
            // a link can't be a child of two other links as URDF models a tree structure and not a graph
            /*
                Here is a snippet from the Pose Frame Semantics documentation for SDFormat that explains the differences
                between URDF and SDF coordinate frame
                http://sdformat.org/tutorials?tut=pose_frame_semantics&ver=1.5#parent-frames-in-urdf
                > The most significant difference between URDF and SDFormat coordinate frames is related to links and joints.
                > While SDFormat allows kinematic loops with the topology of a directed graph,
                > URDF kinematics must have the topology of a directed tree, with each link being the child of at most one joint.
                > URDF coordinate frames are defined recursively based on this tree structure, with each joint's <origin/> tag
                > defining the coordinate transformation from the parent link frame to the child link frame.
            */

            // Use the first joint where this link is a child to locate the parent link pointer.
            const sdf::Joint* joint = jointsWhereLinkIsChild.front();
            std::string parentLinkName = joint->ParentName();
            AZStd::string parentName(parentLinkName.c_str(), parentLinkName.size());

            // Lookup the entity created from the parent link using the JointMapper to locate the parent SDF link.
            // followed by using SDF link address to lookup the O3DE created entity ID
            auto parentLinkIter = jointsMapper.m_jointToParentLinks.find(joint);
            auto parentEntityIter =
                parentLinkIter != jointsMapper.m_jointToParentLinks.end() ? createdLinks.find(parentLinkIter->second) : createdLinks.end();
            if (parentEntityIter == createdLinks.end())
            {
                AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s has invalid parent name %s\n", linkName.c_str(), parentName.c_str());
                continue;
            }
            if (!parentEntityIter->second.IsSuccess())
            {
                AZ_Trace(
                    "CreatePrefabFromUrdfOrSdf",
                    "Link %s has parent %s which has failed to create\n",
                    linkName.c_str(),
                    parentName.c_str());
                continue;
            }
            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf",
                "Link %s setting parent to %s\n",
                linkPrefabResult.GetValue().ToString().c_str(),
                parentEntityIter->second.GetValue().ToString().c_str());
            AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s setting parent to %s\n", linkName.c_str(), parentName.c_str());
            // The joint hierarchy which specifies how a parent and child link hierarchy is represented in an SDF document
            // is used to establish the entity parent child hierarchy, but does not modify the world location of the link entities
            // therefore SetEntityParent is used to maintain the world transform of the child link
            PrefabMakerUtils::SetEntityParent(linkPrefabResult.GetValue(), parentEntityIter->second.GetValue());
        }

        // Iterate over all the joints and locate the entity associated with the link
        for ([[maybe_unused]] const auto& [fullJointName, jointPtr, _] : jointsMapper.m_joints)
        {
            std::string jointName = jointPtr->Name();
            AZStd::string azJointName(jointName.c_str(), jointName.size());
            std::string childLinkName = jointPtr->ChildName();
            std::string parentLinkName = jointPtr->ParentName();

            // Look up the O3DE created entity by first locating the parent SDF link associated with the current joint
            // and then using that SDF link to lookup the created entity
            auto parentLinkIter = jointsMapper.m_jointToParentLinks.find(jointPtr);
            auto parentEntityIter =
                parentLinkIter != jointsMapper.m_jointToParentLinks.end() ? createdLinks.find(parentLinkIter->second) : createdLinks.end();
            if (parentEntityIter == createdLinks.end())
            {
                AZ_Warning(
                    "CreatePrefabFromUrdfOrSdf",
                    false,
                    "Joint %s has no parent link %s. Cannot create",
                    azJointName.c_str(),
                    parentLinkName.c_str());
                continue;
            }
            auto leadEntity = parentEntityIter->second;

            // Use the joint to lookup the child SDF link which is used to look up the O3DE entity
            auto childLinkIter = jointsMapper.m_jointToChildLinks.find(jointPtr);
            auto childEntityIter =
                childLinkIter != jointsMapper.m_jointToChildLinks.end() ? createdLinks.find(childLinkIter->second) : createdLinks.end();
            if (childEntityIter == createdLinks.end())
            {
                AZ_Warning(
                    "CreatePrefabFromUrdfOrSdf",
                    false,
                    "Joint %s has no child link %s. Cannot create",
                    azJointName.c_str(),
                    childLinkName.c_str());
                continue;
            }
            auto childEntity = childEntityIter->second;

            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf",
                "Creating joint %s : %s -> %s\n",
                azJointName.c_str(),
                parentLinkName.c_str(),
                childLinkName.c_str());

            AZ::Entity* childEntityPtr = AzToolsFramework::GetEntityById(childEntity.GetValue());
            if (childEntityPtr)
            {
                auto* component = childEntityPtr->FindComponent<ROS2FrameEditorComponent>();
                if (component)
                {
                    component->SetJointName(azJointName);
                }
            }
            // check if both has RigidBody and we are not creating articulation
            if (!m_useArticulations)
            {
                if (leadEntity.IsSuccess() && childEntity.IsSuccess())
                {
                    AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
                    auto result = m_jointsMaker.AddJointComponent(jointPtr, childEntity.GetValue(), leadEntity.GetValue());
                    if (result.IsSuccess())
                    {
                        m_status.emplace(
                            StatusMessageType::Joint, AZStd::string::format("%s created as: %llu", azJointName.c_str(), result.GetValue()));
                    }
                    else
                    {
                        m_status.emplace(
                            StatusMessageType::Joint,
                            AZStd::string::format("%s failed : %s", azJointName.c_str(), result.GetError().c_str()));
                    }
                }
                else
                {
                    AZ_Warning("CreatePrefabFromUrdfOrSdf", false, "cannot create joint %s", azJointName.c_str());
                }
            }
        }

        // Add control components to links that are not parented to any other link (first link of each model) based on SDFormat data.
        if (!linkEntityIdsWithoutParent.empty())
        {
            for (const auto& [contentEntityId, modelPtr] : linkEntityIdsWithoutParent)
            {
                if (contentEntityId.IsValid())
                {
                    m_controlMaker.AddControlPlugins(*modelPtr, contentEntityId, createdLinks);
                }
            }
        }

        // Get the remaining log information (sensors, plugins)
        {
            AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
            const auto& sensorStatus = m_sensorsMaker.GetStatusMessages();
            for (const auto& ss : sensorStatus)
            {
                m_status.emplace(StatusMessageType::Sensor, ss);
            }
        }

        // Create prefab, save it to disk immediately
        // Remove prefab, if it was already created.

        // clear out any previously created prefab template for this path
        auto* prefabSystemComponentInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabSystemComponentInterface>::Get();
        auto prefabLoaderInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabLoaderInterface>::Get();
        auto relativePath = prefabLoaderInterface->GenerateRelativePath(m_prefabPath.c_str());
        AzToolsFramework::Prefab::TemplateId prefabTemplateId =
            prefabSystemComponentInterface->GetTemplateIdFromFilePath({ relativePath.c_str() });
        if (prefabTemplateId != AzToolsFramework::Prefab::InvalidTemplateId)
        {
            prefabSystemComponentInterface->RemoveTemplate(prefabTemplateId);
            prefabTemplateId = AzToolsFramework::Prefab::InvalidTemplateId;
        }

        prefabTemplateId = AzToolsFramework::Prefab::InvalidTemplateId;

        // create prefab from the "set" of entities (currently just the single default entity)
        AzToolsFramework::Prefab::PrefabSystemScriptingBus::BroadcastResult(
            prefabTemplateId,
            &AzToolsFramework::Prefab::PrefabSystemScriptingBus::Events::CreatePrefabTemplate,
            createdEntities,
            relativePath.String());

        if (prefabTemplateId == AzToolsFramework::Prefab::InvalidTemplateId)
        {
            AZ_Error("CreatePrefabFromUrdfOrSdf", false, "Could not create a prefab template for entities.");
            return AZ::Failure("Could not create a prefab template for entities.");
        }

        AZ_Info("CreatePrefabFromUrdfOrSdf", "Successfully created prefab %s\n", m_prefabPath.c_str());

        return AZ::Success(prefabTemplateId);
    }

    URDFPrefabMaker::CreatePrefabTemplateResult URDFPrefabMaker::CreatePrefabFromUrdfOrSdf()
    {
        // Begin an undo batch for prefab creation process
        AzToolsFramework::UndoSystem::URSequencePoint* currentUndoBatch = nullptr;
        AzToolsFramework::ToolsApplicationRequests::Bus::BroadcastResult(
            currentUndoBatch, &AzToolsFramework::ToolsApplicationRequests::BeginUndoBatch, "Robot Importer prefab creation");
        if (currentUndoBatch == nullptr)
        {
            AZ_Warning("URDF Prefab Maker", false, "Unable to start undobatch, EBus might not be listening");
        }

        auto result = CreatePrefabTemplateFromUrdfOrSdf();
        if (!result.IsSuccess())
        {
            // End undo batch labeled "Robot Importer prefab creation" preemptively if an error occurs
            if (currentUndoBatch != nullptr)
            {
                AzToolsFramework::ToolsApplicationRequests::Bus::Broadcast(
                    &AzToolsFramework::ToolsApplicationRequests::Bus::Events::EndUndoBatch);
            }

            return result;
        }

        auto prefabLoaderInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabLoaderInterface>::Get();

        // Save Template to file
        auto relativePath = prefabLoaderInterface->GenerateRelativePath(m_prefabPath.c_str());
        bool saveResult = prefabLoaderInterface->SaveTemplateToFile(result.GetValue(), m_prefabPath.c_str());
        if (saveResult)
        {
            // If the template saved successfully, also instantiate it into the level.
            auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();
            [[maybe_unused]] auto createPrefabOutcome =
                prefabInterface->InstantiatePrefab(relativePath.String(), AZ::EntityId(), AZ::Vector3::CreateZero());
            if (createPrefabOutcome.IsSuccess())
            {
                MoveEntityToDefaultSpawnPoint(createPrefabOutcome.GetValue(), m_spawnPosition);
            }
        }
        else
        {
            result = AZ::Failure(AZStd::string::format("Could not save the newly created prefab to '%s'", m_prefabPath.c_str()));
        }

        // End undo batch labeled "Robot Importer prefab creation"
        if (currentUndoBatch != nullptr)
        {
            AzToolsFramework::ToolsApplicationRequests::Bus::Broadcast(
                &AzToolsFramework::ToolsApplicationRequests::Bus::Events::EndUndoBatch);
        }

        return result;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::CreateEntityForModel(const sdf::Model& model)
    {
        auto createEntityResult = PrefabMakerUtils::CreateEntity(AZ::EntityId{}, model.Name().c_str());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }

        // Get the model entity and update its transform component with the pose information
        AZ::EntityId entityId = createEntityResult.GetValue();
        AZStd::unique_ptr<AZ::Entity> entity(AzToolsFramework::GetEntityById(entityId));

        if (auto* transformComponent = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
            transformComponent != nullptr)
        {
            gz::math::Pose3d modelPose = model.RawPose();
            AZ::Transform modelTransform = URDF::TypeConversions::ConvertPose(modelPose);
            // Set the local transform for each model to have it be translated in relation
            // to its parent
            transformComponent->SetLocalTM(AZStd::move(modelTransform));
        }

        // Allow the created model entity to persist if there are no errors at ths point
        entity.release();

        return entityId;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::AddEntitiesForLink(
        const sdf::Link& link, const sdf::Model* attachedModel, AZ::EntityId parentEntityId, AZStd::vector<AZ::EntityId>& createdEntities)
    {
        auto createEntityResult = PrefabMakerUtils::CreateEntity(parentEntityId, link.Name().c_str());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }
        AZ::EntityId entityId = createEntityResult.GetValue();
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        createdEntities.emplace_back(entityId);

        const auto frameComponentId = Utils::CreateComponent(entityId, ROS2FrameEditorComponent::TYPEINFO_Uuid());
        if (frameComponentId)
        {
            auto* component = entity->FindComponent<ROS2FrameEditorComponent>();
            AZ_Assert(component, "ROS2 Frame Component does not exist for %s", entityId.ToString().c_str());
            component->SetFrameID(AZStd::string(link.Name().c_str(), link.Name().size()));
        }
        auto createdVisualEntities = m_visualsMaker.AddVisuals(&link, entityId);
        createdEntities.insert(createdEntities.end(), createdVisualEntities.begin(), createdVisualEntities.end());

        if (!m_useArticulations)
        {
            m_inertialsMaker.AddInertial(link.Inertial(), entityId);
        }
        else
        {
            if (attachedModel != nullptr)
            {
                const auto linkResult = m_articulationsMaker.AddArticulationLink(*attachedModel, &link, entityId);
                std::string linkName = link.Name();
                AZStd::string azLinkName(linkName.c_str(), linkName.size());
                if (linkResult.IsSuccess())
                {
                    AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
                    m_status.emplace(
                        StatusMessageType::Joint,
                        AZStd::string::format("%s created as articulation link: %llu", azLinkName.c_str(), linkResult.GetValue()));
                    m_articulationsCounter++;
                }
                else
                {
                    AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
                    m_status.emplace(
                        StatusMessageType::Joint,
                        AZStd::string::format("%s as articulation link failed: %s", azLinkName.c_str(), linkResult.GetError().c_str()));
                }
            }
        }

        if (attachedModel != nullptr)
        {
            m_collidersMaker.AddColliders(*attachedModel, &link, entityId);
            auto createdSensorEntities = m_sensorsMaker.AddSensors(*attachedModel, &link, entityId);
            createdEntities.insert(createdEntities.end(), createdSensorEntities.begin(), createdSensorEntities.end());
        }
        return AZ::Success(entityId);
    }

    const AZStd::string& URDFPrefabMaker::GetPrefabPath() const
    {
        return m_prefabPath;
    }

    void URDFPrefabMaker::MoveEntityToDefaultSpawnPoint(
        const AZ::EntityId& rootEntityId, AZStd::optional<AZ::Transform> spawnPosition = AZStd::nullopt)
    {
        if (!spawnPosition.has_value())
        {
            AZ_Trace("URDF Importer", "SpawnPosition is null - spawning in Editors default position\n");
            return;
        }

        if (auto entity_ = AzToolsFramework::GetEntityById(rootEntityId); entity_ != nullptr)
        {
            auto* transformInterface_ = entity_->FindComponent<AzToolsFramework::Components::TransformComponent>();

            if (transformInterface_ == nullptr)
            {
                AZ_Trace("URDF Importer", "TransformComponent not found in created entity\n") return;
            }

            transformInterface_->SetWorldTM(*spawnPosition);
            AZ_Trace("URDF Importer", "Successfully set spawn position\n")
        }
    }

    AZStd::string URDFPrefabMaker::GetStatus()
    {
        AZStd::string report;
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);

        // Print warnings first
        constexpr unsigned int articulationsLimit = 64;
        if (m_articulationsCounter >= articulationsLimit)
        {
            report += "\n## 💡 Note: the number of articulations (" + AZStd::to_string(m_articulationsCounter) +
                ") might not be supported by the physics engine.\n";
        }

        report += "# The following components were found and parsed:\n";

        const AZStd::unordered_map<StatusMessageType, AZStd::string> names = { { StatusMessageType::Model, "Models" },
                                                                               { StatusMessageType::Link, "Links" },
                                                                               { StatusMessageType::Joint, "Joints" },
                                                                               { StatusMessageType::Sensor, "Sensors" },
                                                                               { StatusMessageType::SensorPlugin, "Sensor plugins" },
                                                                               { StatusMessageType::ModelPlugin, "Model plugins" } };
        auto it = m_status.begin();
        auto end = m_status.end();
        while (it != end)
        {
            const auto key = it->first;
            report += "\n## " + names.at(key) + ":\n";

            do
            {
                report += "- " + it->second + "\n";
                if (++it == end)
                {
                    break;
                }
            } while (it->first == key);
        }
        return report;
    }

    bool URDFPrefabMaker::ContainsModel() const
    {
        const sdf::Model* sdfModel{};
        auto GetModelAndStopIteration = [&sdfModel](const sdf::Model& model, const Utils::ModelStack&) -> Utils::VisitModelResponse
        {
            sdfModel = &model;
            // Return stop to prevent further visitation of additional models
            return Utils::VisitModelResponse::Stop;
        };
        Utils::VisitModels(*m_root, GetModelAndStopIteration);
        return sdfModel != nullptr;
    }
} // namespace ROS2
