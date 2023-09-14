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
#include <AzCore/Debug/Trace.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponentInterface.h>
#include <AzToolsFramework/Prefab/PrefabLoaderScriptingBus.h>
#include <AzToolsFramework/Prefab/PrefabSystemScriptingBus.h>
#include <AzToolsFramework/Prefab/Procedural/ProceduralPrefabAsset.h>
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <optional>

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
        , m_visualsMaker{}
        , m_collidersMaker(urdfAssetsMapping)
        , m_prefabPath(std::move(prefabPath))
        , m_urdfAssetsMapping(urdfAssetsMapping)
        , m_spawnPosition(spawnPosition)
        , m_useArticulations(useArticulations)
    {
        AZ_Assert(!m_prefabPath.empty(), "Prefab path is empty");
        AZ_Assert(m_root, "SDF Root is nullptr");
        if (m_root != nullptr)
        {
            VisualsMaker::MaterialNameMap materialMap;
            auto GetVisualsFromModel = [&materialMap](const sdf::Model& model)
            {
                for (uint64_t linkIndex{}; linkIndex < model.LinkCount(); ++linkIndex)
                {
                    if (const sdf::Link* link = model.LinkByIndex(linkIndex); link != nullptr)
                    {
                        for (uint64_t visualIndex{}; visualIndex < link->VisualCount(); ++visualIndex)
                        {
                            if (const sdf::Visual* visual = link->VisualByIndex(visualIndex); visual != nullptr)
                            {
                                if (const sdf::Material* material = visual->Material(); material != nullptr)
                                {
                                    const std::string visualName = visual->Name();
                                    materialMap.emplace(AZStd::string{ visualName.c_str(), visualName.size() }, material);
                                }
                            }
                        }
                    }
                }
            };

            // Iterate over all visuals to get their materials
            auto VisitAllModels = [&GetVisualsFromModel](const sdf::Model& model) -> Utils::VisitModelResponse
            {
                GetVisualsFromModel(model);
                // Continue to visit all models within the SDF document and query their <visual> tags
                return Utils::VisitModelResponse::VisitNestedAndSiblings;
            };
            Utils::VisitModels(*m_root, VisitAllModels);

            m_visualsMaker = VisualsMaker(AZStd::move(materialMap), urdfAssetsMapping);
        }
    }

    void URDFPrefabMaker::BuildAssetsForLink(const sdf::Link* link)
    {
        m_collidersMaker.BuildColliders(link);

        auto GetAssetsForLinkInModel = [this, link](const sdf::Model& model) -> Utils::VisitModelResponse
        {
            // Find the links which are children in a joint where this link is a parent
            auto BuildAssetsFromJointChildLinks = [this, &model](const sdf::Joint& joint)
            {
                if (const sdf::Link* childLink = model.LinkByName(joint.ChildName()); childLink != nullptr)
                {
                    BuildAssetsForLink(childLink);
                }

                return true;
            };

            // Make sure the link is a child of the model being visited
            // Before visiting the joints of the model
            if (const sdf::Link* searchLink = model.LinkByName(link->Name()); searchLink != nullptr)
            {
                // Don't visit nested models using the VisitJoints function as the outer call
                // to VisitModels already visits nested models
                constexpr bool visitNestedModelLinks = false;
                Utils::VisitJoints(model, BuildAssetsFromJointChildLinks, visitNestedModelLinks);
            }

            return Utils::VisitModelResponse::VisitNestedAndSiblings;
        };

        constexpr bool visitNestedModels = true;
        Utils::VisitModels(*m_root, GetAssetsForLinkInModel, visitNestedModels);
    }

    URDFPrefabMaker::CreatePrefabTemplateResult URDFPrefabMaker::CreatePrefabTemplateFromUrdfOrSdf()
    {
        {
            AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
            m_status.clear();
        }

        if (!ContainsModel())
        {
            return AZ::Failure(AZStd::string("URDF/SDF doesn't contain any models."));
        }

        // Build up a list of all entities created as a part of processing the file.
        AZStd::vector<AZ::EntityId> createdEntities;
        AZStd::unordered_map<AZStd::string, AzToolsFramework::Prefab::PrefabEntityResult> createdLinks;
        AZStd::unordered_map<AZStd::string, const sdf::Link*> links;
        // Gather all links from all the models in the SDF
        auto GetAllLinksFromModel = [&links](const sdf::Model& model) -> Utils::VisitModelResponse
        {
            // As the VisitModels function visits nested models by default, gatherNestedModelLinks is set to false
            constexpr bool gatherNestedModelLinks = false;
            auto linksForModel = Utils::GetAllLinks(model, gatherNestedModelLinks);
            links.insert(linksForModel.begin(), linksForModel.end());

            return Utils::VisitModelResponse::VisitNestedAndSiblings;
        };

        // Visit any nested models in the SDF as well
        constexpr bool visitNestedModels = true;
        Utils::VisitModels(*m_root, GetAllLinksFromModel, visitNestedModels);


        for (const auto& [name, linkPtr] : links)
        {
            createdLinks[name] = AddEntitiesForLink(linkPtr, AZ::EntityId{}, createdEntities);
        }

        for (const auto& [name, result] : createdLinks)
        {
            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf",
                "Link with name %s was created as: %s\n",
                name.c_str(),
                result.IsSuccess() ? (result.GetValue().ToString().c_str()) : ("[Failed]"));
            AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
            if (result.IsSuccess())
            {
                m_status.emplace(name, AZStd::string::format("created as: %s", result.GetValue().ToString().c_str()));
            }
            else
            {
                m_status.emplace(name, AZStd::string::format("failed : %s", result.GetError().c_str()));
            }
        }

        // Set the transforms of links
        for (const auto& [name, linkPtr] : links)
        {
            if (const auto thisEntry = createdLinks.at(name);
                thisEntry.IsSuccess())
            {
                AZ::Transform tf = Utils::GetWorldTransformURDF(linkPtr);
                auto* entity = AzToolsFramework::GetEntityById(thisEntry.GetValue());
                if (entity)
                {
                    auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
                    if (transformInterface)
                    {
                        AZ_Trace(
                            "CreatePrefabFromUrdfOrSdf",
                            "Setting transform %s %s to [%f %f %f] [%f %f %f %f]\n",
                            name.c_str(),
                            thisEntry.GetValue().ToString().c_str(),
                            tf.GetTranslation().GetX(),
                            tf.GetTranslation().GetY(),
                            tf.GetTranslation().GetZ(),
                            tf.GetRotation().GetX(),
                            tf.GetRotation().GetY(),
                            tf.GetRotation().GetZ(),
                            tf.GetRotation().GetW());
                        transformInterface->SetWorldTM(tf);
                    }
                    else
                    {
                        AZ_Trace("CreatePrefabFromUrdfOrSdf", "Setting transform failed: %s does not have transform interface\n", name.c_str());
                    }
                }
            }
        }

        // Set the hierarchy
        AZStd::vector<AZ::EntityId> linkEntityIdsWithoutParent;
        for (const auto& [linkName, linkPtr] : links)
        {
            const auto linkPrefabResult = createdLinks.at(linkName);
            if (!linkPrefabResult.IsSuccess())
            {
                AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s creation failed\n", linkName.c_str());
                continue;
            }

            AZStd::vector<const sdf::Joint*> jointsWhereLinkIsChild;
            if (const sdf::Model* modelContainingLink = Utils::GetModelContainingLink(*m_root, *linkPtr); modelContainingLink != nullptr)
            {
                bool gatherNestedModelJoints = true;
                jointsWhereLinkIsChild = Utils::GetJointsForChildLink(*modelContainingLink, linkName, gatherNestedModelJoints);
            }

            if (jointsWhereLinkIsChild.empty())
            {
                // emplace unique entry to the container of links that don't have a parent link associated with it
                if (auto existingLinkIt =
                        AZStd::find(linkEntityIdsWithoutParent.begin(), linkEntityIdsWithoutParent.end(), linkPrefabResult.GetValue());
                    existingLinkIt == linkEntityIdsWithoutParent.end())
                {
                    linkEntityIdsWithoutParent.emplace_back(linkPrefabResult.GetValue());
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

            AZStd::string parentName(jointsWhereLinkIsChild.front()->ParentName().c_str(),
                jointsWhereLinkIsChild.front()->ParentName().size());
            const auto parentEntry = createdLinks.find(parentName);
            if (parentEntry == createdLinks.end())
            {
                AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s has invalid parent name %s\n", linkName.c_str(), parentName.c_str());
                continue;
            }
            if (!parentEntry->second.IsSuccess())
            {
                AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s has parent %s which has failed to create\n", linkName.c_str(), parentName.c_str());
                continue;
            }
            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf",
                "Link %s setting parent to %s\n",
                linkPrefabResult.GetValue().ToString().c_str(),
                parentEntry->second.GetValue().ToString().c_str());
            AZ_Trace("CreatePrefabFromUrdfOrSdf", "Link %s setting parent to %s\n", linkName.c_str(), parentName.c_str());
            PrefabMakerUtils::SetEntityParent(linkPrefabResult.GetValue(), parentEntry->second.GetValue());
        }

        auto JointVisitor = [this, &createdLinks](const sdf::Joint& joint)
        {
            auto jointPtr = &joint;

            const std::string& jointName = jointPtr->Name();
            AZStd::string azJointName(jointName.c_str(), jointName.size());
            AZStd::string parentLinkName(jointPtr->ParentName().c_str(), jointPtr->ParentName().size());
            AZStd::string childLinkName(jointPtr->ChildName().c_str(), jointPtr->ChildName().size());

            auto parentLinkIter = createdLinks.find(parentLinkName);
            if (parentLinkIter == createdLinks.end())
            {
                AZ_Warning("CreatePrefabFromUrdfOrSdf", false, "Joint %s has no parent link %s. Cannot create", azJointName.c_str(), parentLinkName.c_str());
                return true;
            }
            auto leadEntity = parentLinkIter->second;

            auto childLinkIter = createdLinks.find(childLinkName);
            if (childLinkIter == createdLinks.end())
            {
                AZ_Warning("CreatePrefabFromUrdfOrSdf", false, "Joint %s has no child link %s. Cannot create", azJointName.c_str(), childLinkName.c_str());
                return true;
            }
            auto childEntity = childLinkIter->second;

            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf",
                "Creating joint %s : %s -> %s\n",
                azJointName.c_str(),
                parentLinkName.c_str(),
                childLinkName.c_str());


            AZ::Entity* childEntityPtr = AzToolsFramework::GetEntityById(childEntity.GetValue());
            if (childEntityPtr)
            {
                auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(childEntityPtr);
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
                    m_status.emplace(
                        azJointName, AZStd::string::format(" %s %llu", result.IsSuccess() ? "created as" : "Failed", result.GetValue()));
                }
                else
                {
                    AZ_Warning("CreatePrefabFromUrdfOrSdf", false, "cannot create joint %s", azJointName.c_str());
                }
            }
            return true;
        };

        // Visit all joints that has at least a parent or child link in every model inside of the SDF document
        auto VisitJointsInModel = [&JointVisitor](const sdf::Model& model) -> Utils::VisitModelResponse
        {
            // The JointVisitor is used instead of iterating over the sdf::Model::JointCount
            // as it will skip joints that exist that doesn't have an attached parent link or child link
            // such as the root link when its name is "world"

            // As the VisitModels function visits nested models by default, visitNestedJoints
            // is set to false to prevent visiting joints twice
            constexpr bool visitNestedJoints = false;
            Utils::VisitJoints(model, JointVisitor, visitNestedJoints);
            return Utils::VisitModelResponse::VisitNestedAndSiblings;
        };

        Utils::VisitModels(*m_root, VisitJointsInModel, visitNestedModels);

        // Use the first entity based on a link that is not parented to any other link
        if (!linkEntityIdsWithoutParent.empty() && linkEntityIdsWithoutParent.front().IsValid())
        {
            AZ::EntityId contentEntityId = linkEntityIdsWithoutParent.front();
            MoveEntityToDefaultSpawnPoint(contentEntityId, m_spawnPosition);
            AddRobotControl(contentEntityId);
        }

        // Create prefab, save it to disk immediately
        // Remove prefab, if it was already created.

        AZ::IO::FixedMaxPath prefabTemplateName{ AZ::IO::PathView(m_prefabPath).FixedMaxPathStringAsPosix() };

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
            createdEntities, relativePath.String());

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
        }
        else
        {
            result = AZ::Failure(AZStd::string::format("Could not save the newly created prefab to '%s'",
                m_prefabPath.c_str()));
        }

        // End undo batch labeled "Robot Importer prefab creation"
        if (currentUndoBatch != nullptr)
        {
            AzToolsFramework::ToolsApplicationRequests::Bus::Broadcast(
                &AzToolsFramework::ToolsApplicationRequests::Bus::Events::EndUndoBatch);
        }

        return result;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::AddEntitiesForLink(const sdf::Link* link, AZ::EntityId parentEntityId, AZStd::vector<AZ::EntityId>& createdEntities)
    {
        if (!link)
        {
            AZ::Failure(AZStd::string("Failed to create prefab entity - link is null"));
        }

        auto createEntityResult = PrefabMakerUtils::CreateEntity(parentEntityId, link->Name().c_str());
        if (!createEntityResult.IsSuccess())
        {
            return createEntityResult;
        }
        AZ::EntityId entityId = createEntityResult.GetValue();
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        createdEntities.emplace_back(entityId);

        const auto frameComponentId = Utils::CreateComponent(entityId, ROS2FrameComponent::TYPEINFO_Uuid());
        if (frameComponentId)
        {
            auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
            AZ_Assert(component, "ROS2 Frame Component does not exist for %s", entityId.ToString().c_str());
            component->SetFrameID(AZStd::string(link->Name().c_str(), link->Name().size()));
        }
        auto createdVisualEntities = m_visualsMaker.AddVisuals(link, entityId);
        createdEntities.insert(createdEntities.end(), createdVisualEntities.begin(), createdVisualEntities.end());

        // Get the SDF model where this link is a child
        const sdf::Model* modelContainingLink = Utils::GetModelContainingLink(*m_root, *link);

        if (!m_useArticulations)
        {
            m_inertialsMaker.AddInertial(link->Inertial(), entityId);
        }
        else
        {
            if (modelContainingLink != nullptr)
            {
                m_articulationsMaker.AddArticulationLink(*modelContainingLink, link, entityId);
            }
        }

        if (modelContainingLink != nullptr)
        {
            m_collidersMaker.AddColliders(*modelContainingLink, link, entityId);
        }
        return AZ::Success(entityId);
    }

    void URDFPrefabMaker::AddRobotControl(AZ::EntityId rootEntityId)
    {
        const auto componentId = Utils::CreateComponent(rootEntityId, ROS2RobotControlComponent::TYPEINFO_Uuid());
        if (componentId)
        {
            ControlConfiguration controlConfiguration;
            TopicConfiguration subscriberConfiguration;
            subscriberConfiguration.m_topic = "cmd_vel";
            AZ::Entity* rootEntity = AzToolsFramework::GetEntityById(rootEntityId);
            auto* component = Utils::GetGameOrEditorComponent<ROS2RobotControlComponent>(rootEntity);
            component->SetControlConfiguration(controlConfiguration);
            component->SetSubscriberConfiguration(subscriberConfiguration);
        }
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

        if (auto entity_ = AzToolsFramework::GetEntityById(rootEntityId);
            entity_ != nullptr)
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
        AZStd::string str;
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
        for (const auto& [entry, entryStatus] : m_status)
        {
            str += entry + " " + entryStatus + "\n";
        }
        return str;
    }

    bool URDFPrefabMaker::ContainsModel() const
    {
        const sdf::Model* sdfModel{};
        auto GetModelAndStopIteration = [&sdfModel](const sdf::Model& model) -> Utils::VisitModelResponse
        {
            sdfModel = &model;
            // Return stop to prevent further visitation of additional models
            return Utils::VisitModelResponse::Stop;
        };
        Utils::VisitModels(*m_root, GetModelAndStopIteration);
        return sdfModel != nullptr;
    }
} // namespace ROS2
