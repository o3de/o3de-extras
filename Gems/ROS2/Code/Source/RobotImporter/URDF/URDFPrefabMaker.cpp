/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter/URDF/URDFPrefabMaker.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2GemUtilities.h"
#include "ROS2/Spawner/SpawnerBus.h"
#include "RobotControl/ROS2RobotControlComponent.h"
#include "RobotImporter/URDF/CollidersMaker.h"
#include "RobotImporter/URDF/PrefabMakerUtils.h"
#include "RobotImporter/Utils/RobotImporterUtils.h"
#include <API/EditorAssetSystemAPI.h>
#include <AzCore/IO/FileIO.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponentInterface.h>
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>

namespace ROS2
{
    URDFPrefabMaker::URDFPrefabMaker(const AZStd::string& modelFilePath, urdf::ModelInterfaceSharedPtr model, AZStd::string prefabPath)
        : m_model(model)
        , m_visualsMaker(modelFilePath, model->materials_)
        , m_collidersMaker(modelFilePath)
        , m_prefabPath(std::move(prefabPath))
    {
        AZ_Assert(!m_prefabPath.empty(), "Prefab path is empty");
        AZ_Assert(m_model, "Model is nullptr");
    }

    void URDFPrefabMaker::LoadURDF(BuildReadyCallback buildReadyCb)
    {
        m_notifyBuildReadyCb = buildReadyCb;

        // Request the build of collider meshes by constructing .assetinfo files.
        BuildAssetsForLink(m_model->root_link_);

        // Wait for all collider meshes to be ready
        m_collidersMaker.ProcessMeshes(buildReadyCb);
    }

    void URDFPrefabMaker::BuildAssetsForLink(urdf::LinkSharedPtr link)
    {
        m_collidersMaker.BuildColliders(link);
        for (auto childLink : link->child_links)
        {
            BuildAssetsForLink(childLink);
        }
    }

    AzToolsFramework::Prefab::CreatePrefabResult URDFPrefabMaker::CreatePrefabFromURDF()
    {
        {
            AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
            m_status.clear();
        }
        // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping
        AZStd::unordered_map<AZStd::string, AzToolsFramework::Prefab::PrefabEntityResult> created_links;
        AzToolsFramework::Prefab::PrefabEntityResult createEntityRoot = AddEntitiesForLink(m_model->root_link_, AZ::EntityId());
        AZStd::string root_name(m_model->root_link_->name.c_str(), m_model->root_link_->name.size());
        created_links[root_name] = createEntityRoot;
        if (!createEntityRoot.IsSuccess())
        {
            return AZ::Failure(AZStd::string(createEntityRoot.GetError()));
        }

        auto links = Utils::getAllLinks(m_model->root_link_->child_links);

        // create links
        for (const auto& [name, link_ptr] : links)
        {
            created_links[name] = AddEntitiesForLink(link_ptr, createEntityRoot.GetValue());
        }

        for (const auto& [name, result] : created_links)
        {
            AZ_TracePrintf(
                "CreatePrefabFromURDF",
                "Link with name %s was created as: %s",
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

        // set transforms of links
        for (const auto& [name, link_ptr] : links)
        {
            const auto this_entry = created_links.at(name);
            if (this_entry.IsSuccess())
            {
                AZ::Transform tf = Utils::getWorldTransformURDF(link_ptr);
                auto* entity = AzToolsFramework::GetEntityById(this_entry.GetValue());
                if (entity)
                {
                    auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>();
                    if (transformInterface)
                    {
                        AZ_TracePrintf(
                            "CreatePrefabFromURDF",
                            "Setting transform %s %s to [%f %f %f] [%f %f %f %f]",
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
                            "CreatePrefabFromURDF", "Setting transform failed: %s does not have transform interface", name.c_str());
                    }
                }
            }
        }

        // set hierarchy
        for (const auto& [name, link_ptr] : links)
        {
            const auto this_entry = created_links.at(name);
            if (!this_entry.IsSuccess())
            {
                AZ_TracePrintf("CreatePrefabFromURDF", "Link %s creation failed", name.c_str());
                continue;
            }
            auto parent_ptr = link_ptr->getParent();
            if (!parent_ptr)
            {
                AZ_TracePrintf("CreatePrefabFromURDF", "Link %s has no parents", name.c_str());
                continue;
            }
            AZStd::string parent_name(parent_ptr->name.c_str(), parent_ptr->name.size());
            const auto parent_entry = created_links.find(parent_name);
            if (parent_entry == created_links.end())
            {
                AZ_TracePrintf("CreatePrefabFromURDF", "Link %s has invalid parent name %s", name.c_str(), parent_name.c_str());
                continue;
            }
            if (!parent_entry->second.IsSuccess())
            {
                AZ_TracePrintf(
                    "CreatePrefabFromURDF", "Link %s has parent %s which has failed to create", name.c_str(), parent_name.c_str());
                continue;
            }
            AZ_TracePrintf(
                "CreatePrefabFromURDF",
                "Link %s setting parent to %s",
                this_entry.GetValue().ToString().c_str(),
                parent_entry->second.GetValue().ToString().c_str());
            AZ_TracePrintf("CreatePrefabFromURDF", "Link %s setting parent to %s", name.c_str(), parent_name.c_str());
            auto* entity = AzToolsFramework::GetEntityById(this_entry.GetValue());
            entity->Activate();
            AZ::TransformBus::Event(this_entry.GetValue(), &AZ::TransformBus::Events::SetParent, parent_entry->second.GetValue());
            entity->Deactivate();
        }

        // create joint
        auto joints = Utils::getAllJoints(m_model->root_link_->child_links);
        for (const auto& [name, joint_ptr] : joints)
        {
            AZ_Assert(joint_ptr, "joint %s is null", name.c_str());
            AZ_TracePrintf(
                "CreatePrefabFromURDF",
                "Creating joint %s : %s -> %s",
                name.c_str(),
                joint_ptr->parent_link_name.c_str(),
                joint_ptr->child_link_name.c_str());

            auto lead_entity = created_links.at(joint_ptr->parent_link_name.c_str());
            auto child_entity = created_links.at(joint_ptr->child_link_name.c_str());
            // check if both has RigidBody
            if (lead_entity.IsSuccess() && child_entity.IsSuccess())
            {
                AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
                auto result = m_jointsMaker.AddJointComponent(joint_ptr, child_entity.GetValue(), lead_entity.GetValue());
                if (result.IsSuccess())
                {
                    m_status.emplace(name, AZStd::string::format("created as %llu", result.GetValue()));
                }
                else
                {
                    m_status.emplace(name, AZStd::string::format("Failed:  %s", result.GetError().c_str()));
                }
            }
            else
            {
                AZ_Warning("CreatePrefabFromURDF", false, "cannot create joint %s", name.c_str());
            }
        }

        // move to spawn point
        MoveEntityToDefaultSpawnPoint(createEntityRoot.GetValue());

        auto contentEntityId = createEntityRoot.GetValue();
        AddRobotControl(contentEntityId);

        // Create prefab, save it to disk immediately
        // Remove prefab, if it was already created.

        AZ::EntityId entityId = createEntityRoot.GetValue();

        auto prefabSystemComponent = AZ::Interface<AzToolsFramework::Prefab::PrefabSystemComponentInterface>::Get();
        auto prefabLoaderInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabLoaderInterface>::Get();
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();

        AZ::IO::Path relativeFilePath = prefabLoaderInterface->GenerateRelativePath(m_prefabPath.c_str());

        const auto templateId = prefabSystemComponent->GetTemplateIdFromFilePath(relativeFilePath);
        AZ_TracePrintf("CreatePrefabFromURDF", "GetTemplateIdFromFilePath  %s -> %d \n", m_prefabPath.c_str(), templateId);

        if (templateId != AzToolsFramework::Prefab::InvalidTemplateId)
        {
            AZ_TracePrintf("CreatePrefabFromURDF", "Prefab was already loaded\n");
            prefabSystemComponent->RemoveTemplate(templateId);
        }

        auto outcome = prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList{ entityId }, m_prefabPath.c_str());
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            PrefabMakerUtils::AddRequiredComponentsToEntity(prefabContainerEntityId);
        }
        AZ_TracePrintf("CreatePrefabFromURDF", "Successfully created prefab %s\n", m_prefabPath.c_str());
        return outcome;
    }

    AzToolsFramework::Prefab::PrefabEntityResult URDFPrefabMaker::AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId)
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

        // Add ROS2FrameComponent - TODO: only for top level and joints
        // TODO - add unique namespace to the robot's top level frame
        const auto frameCompontentId = Utils::CreateComponent(entityId, ROS2FrameComponent::TYPEINFO_Uuid());
        if (frameCompontentId)
        {
            auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
            AZ_Assert(component, "ROS2 Frame Component does not exist for %s", entityId.ToString().c_str());
            component->SetFrameID(AZStd::string(link->name.c_str(), link->name.size()));
        }
        m_visualsMaker.AddVisuals(link, entityId);
        m_collidersMaker.AddColliders(link, entityId);
        m_inertialsMaker.AddInertial(link->inertial, entityId);
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
            AZ_TracePrintf("URDF Importer", "Spawner not found - creating entity in (0,0,0)") return;
        }

        auto entity_ = AzToolsFramework::GetEntityById(rootEntityId);
        auto* transformInterface_ = entity_->FindComponent<AzToolsFramework::Components::TransformComponent>();

        if (transformInterface_ == nullptr)
        {
            AZ_TracePrintf("URDF Importer", "TransformComponent not found in created entity") return;
        }

        auto pose = spawner->GetDefaultSpawnPose();

        transformInterface_->SetWorldTM(pose);
    }

    AZStd::string URDFPrefabMaker::getStatus()
    {
        AZStd::string str;
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
        for (const auto& [entry, entry_status] : m_status)
        {
            str += entry + " " + entry_status + "\n";
        }
        return str;
    }
} // namespace ROS2
