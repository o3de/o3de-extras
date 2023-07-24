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
#include <ROS2/Spawner/SpawnerBus.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

namespace ROS2
{
    URDFPrefabMaker::URDFPrefabMaker(
        const AZStd::string& modelFilePath,
        urdf::ModelInterfaceSharedPtr model,
        AZStd::string prefabPath,
        const AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetsMapping,
        bool useArticulations)
        : m_model(model)
        , m_visualsMaker(model->materials_, urdfAssetsMapping)
        , m_collidersMaker(urdfAssetsMapping)
        , m_prefabPath(std::move(prefabPath))
        , m_urdfAssetsMapping(urdfAssetsMapping)
        , m_useArticulations(useArticulations)
    {
        AZ_Assert(!m_prefabPath.empty(), "Prefab path is empty");
        AZ_Assert(m_model, "Model is nullptr");
    }

    void URDFPrefabMaker::BuildAssetsForLink(urdf::LinkSharedPtr link)
    {
        m_collidersMaker.BuildColliders(link);
        for (auto childLink : link->child_links)
        {
            BuildAssetsForLink(childLink);
        }
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabStringFromURDF(AZStd::string& outputPrefab)
    {
        {
            AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
            m_status.clear();
        }

        // Begin an undo batch for prefab creation process
        AzToolsFramework::UndoSystem::URSequencePoint* currentUndoBatch = nullptr;
        AzToolsFramework::ToolsApplicationRequests::Bus::BroadcastResult(
            currentUndoBatch, &AzToolsFramework::ToolsApplicationRequests::BeginUndoBatch, "Robot Importer prefab creation");
        if (currentUndoBatch == nullptr)
        {
            AZ_Warning("URDF Prefab Maker", false, "Unable to start undobatch, EBus might not be listening");
        }

        outputPrefab.clear();

        if (!m_model)
        {
            return AZ::Failure(AZStd::string("Null model."));
        }

        // Build up a list of all entities created as a part of processing the file.
        AZStd::vector<AZ::EntityId> createdEntities;

        AZStd::unordered_map<AZStd::string, AzToolsFramework::Prefab::PrefabEntityResult> createdLinks;
        AzToolsFramework::Prefab::PrefabEntityResult createEntityRoot = AddEntitiesForLink(m_model->root_link_, AZ::EntityId(), createdEntities);
        AZStd::string rootName(m_model->root_link_->name.c_str(), m_model->root_link_->name.size());
        createdLinks[rootName] = createEntityRoot;
        if (!createEntityRoot.IsSuccess())
        {
            // End undo batch labeled "Robot Importer prefab creation" preemptively if an error occurs
            if (currentUndoBatch != nullptr)
            {
                AzToolsFramework::ToolsApplicationRequests::Bus::Broadcast(
                    &AzToolsFramework::ToolsApplicationRequests::Bus::Events::EndUndoBatch);
            }

            return AZ::Failure(AZStd::string(createEntityRoot.GetError()));
        }

        auto links = Utils::GetAllLinks(m_model->root_link_->child_links);

        for (const auto& [name, linkPtr] : links)
        {
            createdLinks[name] = AddEntitiesForLink(linkPtr, createEntityRoot.GetValue(), createdEntities);
        }

        for (const auto& [name, result] : createdLinks)
        {
            AZ_TracePrintf(
                "CreatePrefabFromURDF",
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
            const auto this_entry = createdLinks.at(name);
            if (this_entry.IsSuccess())
            {
                AZ::Transform tf = Utils::GetWorldTransformURDF(linkPtr);
                auto* entity = AzToolsFramework::GetEntityById(this_entry.GetValue());
                if (entity)
                {
                    auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
                    if (transformInterface)
                    {
                        AZ_TracePrintf(
                            "CreatePrefabFromURDF",
                            "Setting transform %s %s to [%f %f %f] [%f %f %f %f]\n",
                            name.c_str(),
                            this_entry.GetValue().ToString().c_str(),
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
                        AZ_TracePrintf(
                            "CreatePrefabFromURDF", "Setting transform failed: %s does not have transform interface\n", name.c_str());
                    }
                }
            }
        }

        // Set the hierarchy
        for (const auto& [name, linkPtr] : links)
        {
            const auto thisEntry = createdLinks.at(name);
            if (!thisEntry.IsSuccess())
            {
                AZ_TracePrintf("CreatePrefabFromURDF", "Link %s creation failed\n", name.c_str());
                continue;
            }
            auto parentPtr = linkPtr->getParent();
            if (!parentPtr)
            {
                AZ_TracePrintf("CreatePrefabFromURDF", "Link %s has no parents\n", name.c_str());
                continue;
            }
            AZStd::string parentName(parentPtr->name.c_str(), parentPtr->name.size());
            const auto parentEntry = createdLinks.find(parentName);
            if (parentEntry == createdLinks.end())
            {
                AZ_TracePrintf("CreatePrefabFromURDF", "Link %s has invalid parent name %s\n", name.c_str(), parentName.c_str());
                continue;
            }
            if (!parentEntry->second.IsSuccess())
            {
                AZ_TracePrintf(
                    "CreatePrefabFromURDF", "Link %s has parent %s which has failed to create\n", name.c_str(), parentName.c_str());
                continue;
            }
            AZ_TracePrintf(
                "CreatePrefabFromURDF",
                "Link %s setting parent to %s\n",
                thisEntry.GetValue().ToString().c_str(),
                parentEntry->second.GetValue().ToString().c_str());
            AZ_TracePrintf("CreatePrefabFromURDF", "Link %s setting parent to %s\n", name.c_str(), parentName.c_str());
            PrefabMakerUtils::SetEntityParent(thisEntry.GetValue(), parentEntry->second.GetValue());
        }


        auto joints = Utils::GetAllJoints(m_model->root_link_->child_links);
        for (const auto& [name, jointPtr] : joints)
        {
            AZ_Assert(jointPtr, "joint %s is null", name.c_str());
            AZ_TracePrintf(
                "CreatePrefabFromURDF",
                "Creating joint %s : %s -> %s\n",
                name.c_str(),
                jointPtr->parent_link_name.c_str(),
                jointPtr->child_link_name.c_str());

            auto leadEntity = createdLinks.at(jointPtr->parent_link_name.c_str());
            auto childEntity = createdLinks.at(jointPtr->child_link_name.c_str());


            AZ::Entity* childEntityPtr = AzToolsFramework::GetEntityById(childEntity.GetValue());
            if (childEntityPtr)
            {
                auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(childEntityPtr);
                if (component)
                {
                    component->SetJointName(AZStd::string(name.c_str(), name.length()));
                }
            }
            // check if both has RigidBody and we are not creating articulation
            if (!m_useArticulations && leadEntity.IsSuccess() && childEntity.IsSuccess())
            {
                AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
                auto result = m_jointsMaker.AddJointComponent(jointPtr, childEntity.GetValue(), leadEntity.GetValue());
                m_status.emplace(
                    name, AZStd::string::format(" %s %llu", result.IsSuccess() ? "created as" : "Failed", result.GetValue()));
            }
            else
            {
                AZ_Warning("CreatePrefabFromURDF", false, "cannot create joint %s", name.c_str());
            }
        }


        MoveEntityToDefaultSpawnPoint(createEntityRoot.GetValue());

        auto contentEntityId = createEntityRoot.GetValue();
        AddRobotControl(contentEntityId);

        // Create prefab, save it to disk immediately
        // Remove prefab, if it was already created.

        AZ::EntityId entityId = createEntityRoot.GetValue();

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
        //AZStd::vector<AZ::EntityId> entities = { entityId };

        // create prefab from the "set" of entities (currently just the single default entity)
        AzToolsFramework::Prefab::PrefabSystemScriptingBus::BroadcastResult(
            prefabTemplateId,
            &AzToolsFramework::Prefab::PrefabSystemScriptingBus::Events::CreatePrefabTemplate,
            createdEntities, relativePath.String());        

        if (prefabTemplateId == AzToolsFramework::Prefab::InvalidTemplateId)
        {
            AZ_Error("SdfAssetBuilderName", false, "Could not create a prefab template for entities.");
            return AZ::Failure("Could not create a prefab template for entities.");
        }
        // Convert the prefab to a JSON string
        AZ::Outcome<AZStd::string, void> outcome;
        AzToolsFramework::Prefab::PrefabLoaderScriptingBus::BroadcastResult(
            outcome, &AzToolsFramework::Prefab::PrefabLoaderScriptingBus::Events::SaveTemplateToString, prefabTemplateId);

        if (outcome.IsSuccess() == false)
        {
            AZ_Error("SdfAssetBuilderName", false, "Could not serialize prefab template as a JSON string");
            return AZ::Failure("Could not serialize prefab template as a JSON string");
        }

        outputPrefab = outcome.GetValue();

        AZ_TracePrintf("CreatePrefabFromURDF", "Successfully created prefab %s\n", m_prefabPath.c_str());

        // End undo batch labeled "Robot Importer prefab creation"
        if (currentUndoBatch != nullptr)
        {
            AzToolsFramework::ToolsApplicationRequests::Bus::Broadcast(
                &AzToolsFramework::ToolsApplicationRequests::Bus::Events::EndUndoBatch);
        }

        return AZ::Success(entityId);
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF()
    {
        AZStd::string outputPrefab;
        auto result = CreatePrefabStringFromURDF(outputPrefab);
        if (!result.IsSuccess())
        {
            return result;
        }

        auto prefabSystemComponent = AZ::Interface<AzToolsFramework::Prefab::PrefabSystemComponentInterface>::Get();
        auto prefabLoaderInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabLoaderInterface>::Get();
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();

        // Save Template to file
        auto relativePath = prefabLoaderInterface->GenerateRelativePath(m_prefabPath.c_str());
        const auto templateId = prefabSystemComponent->GetTemplateIdFromFilePath(relativePath);
        if (!prefabLoaderInterface->SaveTemplateToFile(templateId, m_prefabPath.c_str()))
        {
            return AZ::Failure(AZStd::string::format(
                "CreatePrefabAndSaveToDisk - "
                "Could not save the newly created prefab to file path %s - internal error ",
                m_prefabPath.c_str()));
        }

        [[maybe_unused]] auto createPrefabOutcome = 
            prefabInterface->InstantiatePrefab(relativePath.String(), AZ::EntityId(), AZ::Vector3::CreateZero());

        return result;
    }


    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId, AZStd::vector<AZ::EntityId>& createdEntities)
    {
        if (!link)
        {
            AZ::Failure(AZStd::string("Failed to create prefab entity - link is null"));
        }

        auto createEntityResult = PrefabMakerUtils::CreateEntity(parentEntityId, link->name.c_str());
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
            component->SetFrameID(AZStd::string(link->name.c_str(), link->name.size()));
        }
        auto createdVisualEntities = m_visualsMaker.AddVisuals(link, entityId);
        createdEntities.insert(createdEntities.end(), createdVisualEntities.begin(), createdVisualEntities.end());

        if (!m_useArticulations)
        {
            m_inertialsMaker.AddInertial(link->inertial, entityId);
        }
        else
        {
            m_articulationsMaker.AddArticulationLink(link, entityId);
        }

        m_collidersMaker.AddColliders(link, entityId);
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

    void URDFPrefabMaker::MoveEntityToDefaultSpawnPoint(const AZ::EntityId& rootEntityId)
    {
        auto spawner = ROS2::SpawnerInterface::Get();

        if (spawner == nullptr)
        {
            AZ_TracePrintf("URDF Importer", "Spawner not found - creating entity in (0,0,0)\n") return;
        }

        auto entity_ = AzToolsFramework::GetEntityById(rootEntityId);
        auto* transformInterface_ = entity_->FindComponent<AzToolsFramework::Components::TransformComponent>();

        if (transformInterface_ == nullptr)
        {
            AZ_TracePrintf("URDF Importer", "TransformComponent not found in created entity\n") return;
        }

        auto pose = spawner->GetDefaultSpawnPose();

        transformInterface_->SetWorldTM(pose);
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
} // namespace ROS2
