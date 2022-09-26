/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponent.h"
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    ROS2SpawnerComponent::ROS2SpawnerComponent()
    {
        SpawnerInterface::Register(this);
    }

    ROS2SpawnerComponent::~ROS2SpawnerComponent()
    {
        SpawnerInterface::Unregister(this);
    }

    void ROS2SpawnerComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_getNamesService = ros2Node->create_service<gazebo_msgs::srv::GetWorldProperties>(
            "get_available_spawnable_names",
            [this](const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response)
            {
                this->GetAvailableSpawnableNames(request, response);
            });

        m_spawnService = ros2Node->create_service<gazebo_msgs::srv::SpawnEntity>(
            "spawn_entity",
            [this](const SpawnEntityRequest request, SpawnEntityResponse response)
            {
                this->SpawnEntity(request, response);
            });
    }

    void ROS2SpawnerComponent::Deactivate()
    {
        m_getNamesService.reset();
        m_spawnService.reset();
    }

    void ROS2SpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnerComponent, AZ::Component>()
                ->Version(1)
                ->Field("Spawnables", &ROS2SpawnerComponent::m_spawnables)
                ->Field("Default spawn point", &ROS2SpawnerComponent::m_defaultSpawnPose);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SpawnerComponent>("ROS2 Spawner", "Spawner component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Manages spawning of robots in configurable locations")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ROS2SpawnerComponent::m_spawnables, "Spawnables", "Spawnables")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ROS2SpawnerComponent::m_defaultSpawnPose,
                        "Default spawn pose",
                        "Default spawn pose");
            }
        }
    }

    void ROS2SpawnerComponent::GetAvailableSpawnableNames(
        const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response)
    {
        for (const auto& spawnable : m_spawnables)
        {
            response->model_names.emplace_back(spawnable.first.c_str());
        }
    }

    void ROS2SpawnerComponent::SpawnEntity(const SpawnEntityRequest request, SpawnEntityResponse response)
    {
        AZStd::string_view key(request->name.c_str(), request->name.size());

        if (!m_spawnables.contains(key))
        {
            response->success = false;
            response->status_message = "Requested spawnable name not found";
            return;
        }

        if (!m_tickets.contains(key))
        {
            // if a ticket for this spawnable was not created but the spawnable name is correct, create the ticket and then use it to
            // spawn an entity
            auto spawnable = m_spawnables.find(key);
            m_tickets.emplace(spawnable->first, AzFramework::EntitySpawnTicket(spawnable->second));
        }

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();

        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

        optionalArgs.m_preInsertionCallback = [this, position = request->initial_pose](auto id, auto view)
        {
            this->PreSpawn(
                id,
                view,
                AZ::Transform(
                    AZ::Vector3(position.position.x, position.position.y, position.position.z),
                    AZ::Quaternion(position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w),
                    1.0f));
        };

        spawner->SpawnAllEntities(m_tickets.at(key), optionalArgs);

        response->success = true;
    }

    void ROS2SpawnerComponent::PreSpawn(
        AzFramework::EntitySpawnTicket::Id id [[maybe_unused]],
        AzFramework::SpawnableEntityContainerView view,
        const AZ::Transform& transform)
    {
        if (view.empty())
        {
            return;
        }

        AZ::Entity* root = *view.begin();

        // TODO: probably it would be better to use TransformBus here
        auto* transformInterface_ = root->FindComponent<AzFramework::TransformComponent>();

        transformInterface_->SetWorldTM(transform);
    }

    const AZ::Transform& ROS2SpawnerComponent::GetDefaultSpawnPose() const
    {
        return m_defaultSpawnPose;
    }
} // namespace ROS2
