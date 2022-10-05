/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2SpawnPointComponent.h"
#include "SpawnerBus.h"
#include <AzCore/Component/Component.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <gazebo_msgs/srv/get_model_state.hpp>
#include <gazebo_msgs/srv/get_world_properties.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ROS2
{
    using GetAvailableSpawnableNamesRequest = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Request>;
    using GetAvailableSpawnableNamesResponse = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Response>;
    using SpawnEntityRequest = std::shared_ptr<gazebo_msgs::srv::SpawnEntity::Request>;
    using SpawnEntityResponse = std::shared_ptr<gazebo_msgs::srv::SpawnEntity::Response>;
    using GetSpawnPointInfoRequest = std::shared_ptr<gazebo_msgs::srv::GetModelState::Request>;
    using GetSpawnPointInfoResponse = std::shared_ptr<gazebo_msgs::srv::GetModelState::Response>;
    using GetSpawnPointsNamesRequest = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Request>;
    using GetSpawnPointsNamesResponse = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Response>;

    //! Manages robots spawning.
    //! Allows user to set spawnable prefabs in the Editor and spawn them using ROS2 service during the simulation.
    class ROS2SpawnerComponent
        : public AZ::Component
        , public SpawnerRequestsBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2SpawnerComponent, "{5950AC6B-75F3-4E0F-BA5C-17C877013710}", AZ::Component, SpawnerRequestsBus::Handler);

        // AZ::Component interface implementation.
        ROS2SpawnerComponent();
        ~ROS2SpawnerComponent();
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

        const AZ::Transform& GetDefaultSpawnPose() const override;

    private:
        AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> m_spawnables;
        AZStd::unordered_map<AZStd::string, AzFramework::EntitySpawnTicket> m_tickets;

        rclcpp::Service<gazebo_msgs::srv::GetWorldProperties>::SharedPtr m_getSpawnablesNamesService;
        rclcpp::Service<gazebo_msgs::srv::GetWorldProperties>::SharedPtr m_getSpawnPointsNamesService;
        rclcpp::Service<gazebo_msgs::srv::SpawnEntity>::SharedPtr m_spawnService;
        rclcpp::Service<gazebo_msgs::srv::GetModelState>::SharedPtr m_getSpawnPointInfoService;

        AZ::Transform m_defaultSpawnPose = { AZ::Vector3{ 0, 0, 0 }, AZ::Quaternion{ 0, 0, 0, 1 }, 1.0 };

        void GetAvailableSpawnableNames(const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response);
        void SpawnEntity(const SpawnEntityRequest request, SpawnEntityResponse response);
        void PreSpawn(
            AzFramework::EntitySpawnTicket::Id,
            AzFramework::SpawnableEntityContainerView,
            const AZ::Transform&,
            const AZStd::string& spawnable_name);

        void GetSpawnPointsNames(const GetSpawnPointsNamesRequest request, GetSpawnPointsNamesResponse response);
        void GetSpawnPointInfo(const GetSpawnPointInfoRequest request, GetSpawnPointInfoResponse response);

        AZStd::unordered_map<AZStd::string, SpawnPointInfo> GetSpawnPoints();
    };
} // namespace ROS2
