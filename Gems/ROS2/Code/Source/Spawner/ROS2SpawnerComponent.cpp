/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponent.h"
#include "Spawner/ROS2SpawnerComponentController.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/conversions.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{

    ROS2SpawnerComponent::ROS2SpawnerComponent(const ROS2SpawnerComponentConfig& properties)
        : ROS2SpawnerComponentBase(properties)
    {
    }

    void ROS2SpawnerComponent::Activate()
    {
        ROS2SpawnerComponentBase::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "ROS 2 node is not initialized");

        m_getSpawnablesNamesService = ros2Node->create_service<gazebo_msgs::srv::GetWorldProperties>(
            "get_available_spawnable_names",
            [this](const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response)
            {
                GetAvailableSpawnableNames(request, response);
            });

        m_spawnService = ros2Node->create_service<gazebo_msgs::srv::SpawnEntity>(
            "spawn_entity",
            [this](
                const SpawnEntityServiceHandle service_handle,
                const std::shared_ptr<rmw_request_id_t> header,
                const SpawnEntityRequest request)
            {
                SpawnEntity(service_handle, header, request);
            });

        m_deleteService = ros2Node->create_service<gazebo_msgs::srv::DeleteEntity>(
            "delete_entity",
            [this](
                const DeleteEntityServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, DeleteEntityRequest request)
            {
                DeleteEntity(service_handle, header, request);
            });

        m_getSpawnPointInfoService = ros2Node->create_service<gazebo_msgs::srv::GetModelState>(
            "get_spawn_point_info",
            [this](const GetSpawnPointInfoRequest request, GetSpawnPointInfoResponse response)
            {
                GetSpawnPointInfo(request, response);
            });

        m_getSpawnPointsNamesService = ros2Node->create_service<gazebo_msgs::srv::GetWorldProperties>(
            "get_spawn_points_names",
            [this](const GetSpawnPointsNamesRequest request, GetSpawnPointsNamesResponse response)
            {
                GetSpawnPointsNames(request, response);
            });
    }

    void ROS2SpawnerComponent::Deactivate()
    {
        ROS2SpawnerComponentBase::Deactivate();

        m_getSpawnablesNamesService.reset();
        m_spawnService.reset();
        m_deleteService.reset();
        m_getSpawnPointInfoService.reset();
        m_getSpawnPointsNamesService.reset();
        m_tickets.clear();
    }

    void ROS2SpawnerComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnerComponentBase::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnerComponent, ROS2SpawnerComponentBase>()->Version(1);
        }
    }

    void ROS2SpawnerComponent::GetAvailableSpawnableNames(
        const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response)
    {
        for (const auto& spawnable : m_controller.GetSpawnables())
        {
            response->model_names.emplace_back(spawnable.first.c_str());
        }
    }

    void ROS2SpawnerComponent::SpawnEntity(
        const SpawnEntityServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, const SpawnEntityRequest request)
    {
        AZStd::string spawnableName(request->name.c_str());
        AZStd::string spawnableNamespace(request->robot_namespace.c_str());
        AZStd::string spawnPointName(request->xml.c_str(), request->xml.size());

        SpawnEntityResponse response;

        if (auto namespaceValidation = ROS2Names::ValidateNamespace(spawnableNamespace); !namespaceValidation.IsSuccess())
        {
            response.success = false;
            response.status_message = namespaceValidation.GetError().data();
            service_handle->send_response(*header, response);
            return;
        }

        if (!m_controller.GetSpawnables().contains(spawnableName))
        {
            response.success = false;
            response.status_message = "Could not find spawnable with given name: " + request->name;
            service_handle->send_response(*header, response);
            return;
        }

        auto spawnable = m_controller.GetSpawnables().find(spawnableName);

        if (spawnable->second->IsLoading())
        {
            // This is an Editor only situation. All assets during game mode are fully loaded.
            response.success = false;
            response.status_message = "Asset for spawnable " + request->name + " has not yet loaded.";
            service_handle->send_response(*header, response);
            return;
        }

        if (spawnable->second->IsError())
        {
            response.success = false;
            response.status_message = "Spawnable " + request->name + " loaded with an error.";
            service_handle->send_response(*header, response);
            return;
        }

        if (!m_tickets.contains(spawnableName))
        {
            // if a ticket for this spawnable was not created but the spawnable name is correct, create the ticket and then use it to
            // spawn an entity
            m_tickets.emplace(spawnable->first, AzFramework::EntitySpawnTicket(spawnable->second));
        }

        auto spawnableTicket = AzFramework::EntitySpawnTicket(spawnable->second);
        auto ticketId = spawnableTicket.GetId();
        AZStd::string ticketName = spawnable->first + "_" + AZStd::to_string(ticketId);
        m_tickets.emplace(ticketName, AZStd::move(spawnableTicket));

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();

        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;

        AZ::Transform transform;

        if (auto spawnPoints = GetSpawnPoints(); spawnPoints.contains(spawnPointName))
        {
            transform = spawnPoints.at(spawnPointName).pose;
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

        optionalArgs.m_preInsertionCallback = [this, transform, spawnableName, spawnableNamespace](auto id, auto view)
        {
            PreSpawn(id, view, transform, spawnableName, spawnableNamespace);
        };

        optionalArgs.m_completionCallback = [service_handle, header, ticketName](auto id, auto view)
        {
            SpawnEntityResponse response;
            response.success = true;
            response.status_message = ticketName.c_str();
            service_handle->send_response(*header, response);
        };

        spawner->SpawnAllEntities(m_tickets.at(ticketName), optionalArgs);
    }

    void ROS2SpawnerComponent::PreSpawn(
        AzFramework::EntitySpawnTicket::Id id [[maybe_unused]],
        AzFramework::SpawnableEntityContainerView view,
        const AZ::Transform& transform,
        const AZStd::string& spawnableName,
        const AZStd::string& spawnableNamespace)
    {
        if (view.empty())
        {
            return;
        }
        AZ::Entity* root = *view.begin();

        auto* transformInterface = root->FindComponent<AzFramework::TransformComponent>();
        transformInterface->SetWorldTM(transform);

        AZStd::string instanceName = AZStd::string::format("%s_%d", spawnableName.c_str(), m_counter++);
        for (AZ::Entity* entity : view)
        { // Update name for the first entity with ROS2Frame in hierarchy (left to right)
            auto* frameComponent = entity->FindComponent<ROS2FrameComponent>();
            if (frameComponent)
            {
                entity->SetName(instanceName);
                if (!spawnableNamespace.empty())
                {
                    frameComponent->UpdateNamespaceConfiguration(spawnableNamespace, NamespaceConfiguration::NamespaceStrategy::Custom);
                }
                break;
            }
        }
    }

    void ROS2SpawnerComponent::DeleteEntity(
        const DeleteEntityServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, DeleteEntityRequest request)
    {
        auto deleteName = AZStd::string(request->name.c_str());
        if (!m_tickets.contains(deleteName))
        {
            DeleteEntityResponse response;
            response.success = false;
            response.status_message = "Could not find entity with given name: " + request->name;
            service_handle->send_response(*header, response);
            return;
        }
        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();

        AzFramework::DespawnAllEntitiesOptionalArgs optionalArgs;

        optionalArgs.m_completionCallback = [service_handle, header](auto id)
        {
            DeleteEntityResponse response;
            response.success = true;
            service_handle->send_response(*header, response);
        };

        spawner->DespawnAllEntities(m_tickets.at(deleteName), optionalArgs);
        m_tickets.erase(deleteName);
    }

    void ROS2SpawnerComponent::GetSpawnPointsNames(
        const ROS2::GetSpawnPointsNamesRequest request, ROS2::GetSpawnPointsNamesResponse response)
    {
        for (auto spawnPoint : GetSpawnPoints())
        {
            response->model_names.emplace_back(spawnPoint.first.c_str());
        }
    }

    void ROS2SpawnerComponent::GetSpawnPointInfo(const ROS2::GetSpawnPointInfoRequest request, ROS2::GetSpawnPointInfoResponse response)
    {
        const AZStd::string_view key(request->model_name.c_str(), request->model_name.size());

        auto spawnPoints = GetSpawnPoints();
        if (spawnPoints.contains(key))
        {
            auto info = spawnPoints.at(key);
            response->pose = ROS2Conversions::ToROS2Pose(info.pose);
            response->status_message = info.info.c_str();
        }
        else
        {
            response->status_message = "Could not find spawn point with given name: " + request->model_name;
        }
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerComponent::GetSpawnPoints()
    {
        return m_controller.GetSpawnPoints();
    }
} // namespace ROS2
