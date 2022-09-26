/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <gazebo_msgs/srv/get_world_properties.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ROS2
{
    using GetAvailableSpawnableNamesRequest = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Request>;
    using GetAvailableSpawnableNamesResponse = std::shared_ptr<gazebo_msgs::srv::GetWorldProperties::Response>;
    using SpawnEntityRequest = std::shared_ptr<gazebo_msgs::srv::SpawnEntity::Request>;
    using SpawnEntityResponse = std::shared_ptr<gazebo_msgs::srv::SpawnEntity::Response>;

    //! Manages robots spawning.
    //! Allows user to set spawnable prefabs in the Editor and spawn them using ROS2 service during the simulation.
    class ROS2SpawnerComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(ROS2SpawnerComponent, "{5950AC6B-75F3-4E0F-BA5C-17C877013710}", AZ::Component);

        // AZ::Component interface implementation.
        ROS2SpawnerComponent() = default;

        ~ROS2SpawnerComponent() = default;

        void Activate() override;

        void Deactivate() override;

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

    private:
        AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> m_spawnables;
        AZStd::unordered_map<AZStd::string, AzFramework::EntitySpawnTicket> m_tickets;

        rclcpp::Service<gazebo_msgs::srv::GetWorldProperties>::SharedPtr m_getNamesService;
        rclcpp::Service<gazebo_msgs::srv::SpawnEntity>::SharedPtr m_spawnService;

        void GetAvailableSpawnableNames(const GetAvailableSpawnableNamesRequest request, GetAvailableSpawnableNamesResponse response);

        void SpawnEntity(const SpawnEntityRequest request, SpawnEntityResponse response);

        void PreSpawn(AzFramework::EntitySpawnTicket::Id, AzFramework::SpawnableEntityContainerView, const AZ::Transform&);
    };
} // namespace ROS2
