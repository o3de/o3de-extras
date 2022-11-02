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

#include <Spawner/SpawnerBus.h>

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
    { // TODO - this is PoC code, restructure when developing semantics of URDF->Prefab/Entities/Components mapping

        // recursively add all entities
        auto createEntityResult = AddEntitiesForLink(m_model->root_link_, AZ::EntityId());
        if (!createEntityResult.IsSuccess())
        {
            return AZ::Failure(AZStd::string(createEntityResult.GetError()));
        }

        auto contentEntityId = createEntityResult.GetValue();
        AddRobotControl(contentEntityId);

        // Create prefab, save it to disk immediately
        // Remove prefab, if it was already created.

        AZ::EntityId entityId = createEntityResult.GetValue();

        auto prefabSystemComponent = AZ::Interface<AzToolsFramework::Prefab::PrefabSystemComponentInterface>::Get();
        auto prefabLoaderInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabLoaderInterface>::Get();
        auto prefabInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabPublicInterface>::Get();

        AZ::IO::Path relativeFilePath = prefabLoaderInterface->GenerateRelativePath(m_prefabPath.c_str());

        const auto templateId = prefabSystemComponent->GetTemplateIdFromFilePath(relativeFilePath);
        AZ_TracePrintf("CreatePrefabFromURDF", "GetTemplateIdFromFilePath  %s -> %d \n", m_prefabPath.c_str(), templateId);

        if (templateId != AzToolsFramework::Prefab::InvalidTemplateId)
        {
            AZ_TracePrintf("CreatePrefabFromURDF", "Prefab was already loaded \n");
            prefabSystemComponent->RemoveTemplate(templateId);
        }

        auto outcome = prefabInterface->CreatePrefabInDisk(AzToolsFramework::EntityIdList{ entityId }, m_prefabPath.c_str());
        if (outcome.IsSuccess())
        {
            AZ::EntityId prefabContainerEntityId = outcome.GetValue();
            PrefabMakerUtils::AddRequiredComponentsToEntity(prefabContainerEntityId);
        }
        AZ_TracePrintf("CreatePrefabFromURDF", "Successfully created %s prefab\n", m_prefabPath.c_str());
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
            AZ_Assert(component, "Component not exists for %s", entityId.ToString().c_str());
            component->SetFrameID(AZStd::string(link->name.c_str(), link->name.size()));
        }
        m_visualsMaker.AddVisuals(link, entityId);
        m_collidersMaker.AddColliders(link, entityId);
        m_inertialsMaker.AddInertial(link->inertial, entityId);

        for (auto childLink : link->child_links)
        {
            auto outcome = AddEntitiesForLink(childLink, entityId); // recursive call
            if (!outcome.IsSuccess())
            { // TODO - decide on behavior. Still proceed to load other children?
                AZ_Warning("AddEntitiesForLink", false, "Unable to add entity due to an error: %s", outcome.GetError().c_str());
                continue;
            }

            AZ::EntityId childEntityId = outcome.GetValue();
            m_jointsMaker.AddJoint(link, childLink, childEntityId, entityId);
        }

        MoveEntityToDefaultSpawnPoint(entityId);

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
} // namespace ROS2
