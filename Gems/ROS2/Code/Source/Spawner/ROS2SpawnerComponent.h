/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2SpawnPointComponent.h"
#include "Spawner/ROS2SpawnerComponentController.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Component/Component.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include <gazebo_msgs/srv/get_model_state.hpp>
#include <gazebo_msgs/srv/get_world_properties.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ROS2
{
    using GetAvailableSpawnableNamesRequest = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Request>;
    using GetAvailableSpawnableNamesResponse = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Response>;
    using SpawnEntityRequest = std::shared_ptr<gazebo_msgs::srv::SpawnEntity::Request>;
    using SpawnEntityResponse = gazebo_msgs::srv::SpawnEntity::Response;
    using SpawnEntityServiceHandle = std::shared_ptr<rclcpp::Service<gazebo_msgs::srv::SpawnEntity>>;
    using DeleteEntityRequest = std::shared_ptr<gazebo_msgs::srv::DeleteEntity::Request>;
    using DeleteEntityServiceHandle = std::shared_ptr<rclcpp::Service<gazebo_msgs::srv::DeleteEntity>>;
    using DeleteEntityResponse = gazebo_msgs::srv::DeleteEntity::Response;
    using GetSpawnPointInfoRequest = std::shared_ptr<gazebo_msgs::srv::GetModelState::Request>;
    using GetSpawnPointInfoResponse = std::shared_ptr<gazebo_msgs::srv::GetModelState::Response>;
    using GetSpawnPointsNamesRequest = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Request>;
    using GetSpawnPointsNamesResponse = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Response>;

    using ROS2SpawnerComponentBase = AzFramework::Components::ComponentAdapter<ROS2SpawnerComponentController, ROS2SpawnerComponentConfig>;
    //! Manages robots spawning.
    //! Allows user to set spawnable prefabs in the Editor and spawn them using ROS2 service during the simulation.
    class ROS2SpawnerComponent : public ROS2SpawnerComponentBase
    {
    public:
        AZ_COMPONENT(ROS2SpawnerComponent, "{8ea91880-0067-11ee-be56-0242ac120002}", AZ::Component);

        // ROS2SpawnerComponentBase interface implementation.
        ROS2SpawnerComponent() = default;
        ROS2SpawnerComponent(const ROS2SpawnerComponentConfig& properties);
        ~ROS2SpawnerComponent() = default;
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void Reflect(AZ::ReflectContext* context);

    private:
        int m_counter = 1;
        AZStd::unordered_map<AZStd::string, AzFramework::EntitySpawnTicket> m_tickets;

        rclcpp::Service<gazebo_msgs::srv::GetWorldProperties>::SharedPtr m_getSpawnablesNamesService;
        rclcpp::Service<gazebo_msgs::srv::GetWorldProperties>::SharedPtr m_getSpawnPointsNamesService;
        rclcpp::Service<gazebo_msgs::srv::SpawnEntity>::SharedPtr m_spawnService;
        rclcpp::Service<gazebo_msgs::srv::DeleteEntity>::SharedPtr m_deleteService;
        rclcpp::Service<gazebo_msgs::srv::GetModelState>::SharedPtr m_getSpawnPointInfoService;

        void GetAvailableSpawnableNames(const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response);
        void SpawnEntity(
            const SpawnEntityServiceHandle service_handle,
            const std::shared_ptr<rmw_request_id_t> header,
            const SpawnEntityRequest request);

        void PreSpawn(
            AzFramework::EntitySpawnTicket::Id,
            AzFramework::SpawnableEntityContainerView,
            const AZ::Transform&,
            const AZStd::string& spawnableName,
            const AZStd::string& spawnableNamespace);

        void DeleteEntity(
            const DeleteEntityServiceHandle service_handle, const std::shared_ptr<rmw_request_id_t> header, DeleteEntityRequest request);

        void GetSpawnPointsNames(const GetSpawnPointsNamesRequest request, GetSpawnPointsNamesResponse response);
        void GetSpawnPointInfo(const GetSpawnPointInfoRequest request, GetSpawnPointInfoResponse response);

        AZStd::unordered_map<AZStd::string, SpawnPointInfo> GetSpawnPoints();
    };
} // namespace ROS2
