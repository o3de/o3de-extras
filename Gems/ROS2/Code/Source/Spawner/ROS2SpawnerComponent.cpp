/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponent.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "ROS2/ROS2GemUtilities.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

namespace ROS2
{
    ROS2SpawnerComponent::ROS2SpawnerComponent()
    {
        // TODO - currently causes errors on close. It is here to enable URDF spawning in default point.
        // SpawnerInterface::Register(this);
    }

    ROS2SpawnerComponent::~ROS2SpawnerComponent()
    {
        // SpawnerInterface::Unregister(this);
    }

    void ROS2SpawnerComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();

        m_getSpawnablesNamesService = ros2Node->create_service<gazebo_msgs::srv::GetWorldProperties>(
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

        m_getSpawnPointInfoService = ros2Node->create_service<gazebo_msgs::srv::GetModelState>(
            "get_spawn_point_info",
            [this](const GetSpawnPointInfoRequest request, GetSpawnPointInfoResponse response)
            {
                this->GetSpawnPointInfo(request, response);
            });

        m_getSpawnPointsNamesService = ros2Node->create_service<gazebo_msgs::srv::GetWorldProperties>(
            "get_spawn_points_names",
            [this](const GetSpawnPointsNamesRequest request, GetSpawnPointsNamesResponse response)
            {
                this->GetSpawnPointsNames(request, response);
            });
    }

    void ROS2SpawnerComponent::Deactivate()
    {
        m_getSpawnablesNamesService.reset();
        m_spawnService.reset();
        m_getSpawnPointInfoService.reset();
        m_getSpawnPointsNamesService.reset();
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
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
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
        AZStd::string_view spawnable_name(request->name.c_str(), request->name.size());
        // xml parameter of the request is used here like a regular string and stores name of a spawn point
        // todo: use xml format in this parameter
        AZStd::string_view spawn_point_name(request->xml.c_str(), request->xml.size());

        auto spawn_points = GetSpawnPoints();

        if (!m_spawnables.contains(spawnable_name))
        {
            response->success = false;
            response->status_message = "Could not find spawnable with given name: " + request->name;
            return;
        }

        if (!m_tickets.contains(spawnable_name))
        {
            // if a ticket for this spawnable was not created but the spawnable name is correct, create the ticket and then use it to
            // spawn an entity
            auto spawnable = m_spawnables.find(spawnable_name);
            m_tickets.emplace(spawnable->first, AzFramework::EntitySpawnTicket(spawnable->second));
        }

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();

        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

        AZ::Transform transform;

        if (spawn_points.contains(spawn_point_name))
        {
            transform = spawn_points.at(spawn_point_name).pose;
        }
        else
        {
            transform = { AZ::Vector3(request->initial_pose.position.x, request->initial_pose.position.y, request->initial_pose.position.z),
                          AZ::Quaternion(
                              request->initial_pose.orientation.x,
                              request->initial_pose.orientation.y,
                              request->initial_pose.orientation.z,
                              request->initial_pose.orientation.w),
                          1.0f };
        }

        optionalArgs.m_preInsertionCallback = [this, transform, spawnable_name](auto id, auto view)
        {
            this->PreSpawn(id, view, transform, spawnable_name);
        };

        spawner->SpawnAllEntities(m_tickets.at(spawnable_name), optionalArgs);

        response->success = true;
    }

    void ROS2SpawnerComponent::PreSpawn(
        AzFramework::EntitySpawnTicket::Id id [[maybe_unused]],
        AzFramework::SpawnableEntityContainerView view,
        const AZ::Transform& transform,
        const AZStd::string& spawnable_name)
    {
        if (view.empty())
        {
            return;
        }
        AZ::Entity* root = *view.begin();

        // TODO: probably it would be better to use TransformBus here
        auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
        transformInterface->SetWorldTM(transform);

        static int counter = 1;
        AZStd::string instanceName = AZStd::string::format("%s_%d", spawnable_name.c_str(), counter++);
        for (AZ::Entity* entity : view)
        { // Update name for the first entity with ROS2Frame in hierarchy (left to right)
            const auto* frameComponent = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
            if (frameComponent)
            {
                entity->SetName(instanceName);
                break;
            }
        }
    }

    const AZ::Transform& ROS2SpawnerComponent::GetDefaultSpawnPose() const
    {
        return m_defaultSpawnPose;
    }

    void ROS2SpawnerComponent::GetSpawnPointsNames(
        const ROS2::GetSpawnPointsNamesRequest request, ROS2::GetSpawnPointsNamesResponse response)
    {
        for (auto spawn_point : GetSpawnPoints())
        {
            response->model_names.emplace_back(spawn_point.first.c_str());
        }
    }

    void ROS2SpawnerComponent::GetSpawnPointInfo(const ROS2::GetSpawnPointInfoRequest request, ROS2::GetSpawnPointInfoResponse response)
    {
        const AZStd::string_view key(request->model_name.c_str(), request->model_name.size());

        auto spawn_points = GetSpawnPoints();
        if (spawn_points.contains(key))
        {
            auto info = spawn_points.at(key);

            geometry_msgs::msg::Pose pose;
            pose.position.x = info.pose.GetTranslation().GetX();
            pose.position.y = info.pose.GetTranslation().GetY();
            pose.position.z = info.pose.GetTranslation().GetZ();
            pose.orientation.x = info.pose.GetRotation().GetX();
            pose.orientation.y = info.pose.GetRotation().GetY();
            pose.orientation.z = info.pose.GetRotation().GetZ();
            pose.orientation.w = info.pose.GetRotation().GetW();

            response->pose = pose;
            response->status_message = info.info.c_str();
        }
        else
        {
            response->status_message = "Could not find spawn point with given name: " + request->model_name;
        }
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerComponent::GetSpawnPoints()
    {
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, GetEntityId(), &AZ::TransformBus::Events::GetChildren);

        AZStd::unordered_map<AZStd::string, SpawnPointInfo> result;

        for (const auto child : children)
        {
            auto child_entity = AzToolsFramework::GetEntityById(child);

            auto spawn_point = child_entity->FindComponent<ROS2SpawnPointComponent>();

            if (spawn_point == nullptr)
            {
                continue;
            }

            result.insert(spawn_point->GetInfo());
        }

        // setting name of spawn point component "default" in a child entity will have no effect since it is overwritten here with the
        // default spawn pose of spawner
        result["default"] = SpawnPointInfo{ "Default spawn pose defined in the Editor", m_defaultSpawnPose };
        return result;
    }
} // namespace ROS2
