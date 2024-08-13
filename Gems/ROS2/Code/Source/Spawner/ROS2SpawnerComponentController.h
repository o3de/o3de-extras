/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2/Spawner/SpawnerBus.h"
#include "ROS2SpawnPointComponent.h"
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Memory/Memory_fwd.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/base.h>
#include <AzFramework/Spawnable/Spawnable.h>

namespace ROS2
{

    struct ROS2SpawnerServiceNames
    {
        AZ_TYPE_INFO(ROS2SpawnerServiceNames, "{10D75AAC-BD51-4F33-BCAA-9001CFA219AE}");
        AZStd::string m_availableSpawnableNamesServiceName = "get_available_spawnable_names";
        AZStd::string m_spawnEntityServiceName = "spawn_entity";
        AZStd::string m_deleteEntityServiceName = "delete_entity";
        AZStd::string m_spawnPointInfoServiceName = "get_spawn_point_info";
        AZStd::string m_spawnPointsNamesServiceName = "get_spawn_points_names";

        static void Reflect(AZ::ReflectContext* context);
    };

    class ROS2SpawnerComponentConfig final : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(ROS2SpawnerComponentConfig, "{ee71f892-006a-11ee-be56-0242ac120002}");

        ROS2SpawnerComponentConfig() = default;
        ~ROS2SpawnerComponentConfig() = default;

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_editorEntityId;
        AZ::Transform m_defaultSpawnPose = { AZ::Vector3{ 0, 0, 0 }, AZ::Quaternion{ 0, 0, 0, 1 }, 1.0 };

        ROS2SpawnerServiceNames m_serviceNames;
        AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> m_spawnables;
        bool m_supportWGS{ true };
    };

    class ROS2SpawnerComponentController : public SpawnerRequestsBus::Handler
    {
    public:
        AZ_TYPE_INFO(ROS2SpawnerComponentController, "{1e9e040c-006b-11ee-be56-0242ac120002}");
        ROS2SpawnerComponentController() = default;
        explicit ROS2SpawnerComponentController(const ROS2SpawnerComponentConfig& config);

        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // Controller component
        void Init();
        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const ROS2SpawnerComponentConfig& config);
        const ROS2SpawnerComponentConfig& GetConfiguration() const;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // SpawnerRequestsBus::Handler overrides
        const AZ::Transform& GetDefaultSpawnPose() const override;
        SpawnPointInfoMap GetAllSpawnPointInfos() const override;
        //////////////////////////////////////////////////////////////////////////

        const ROS2SpawnerServiceNames& GetServiceNames() const;
        SpawnPointInfoMap GetSpawnPoints() const;
        AZ::EntityId GetEditorEntityId() const;
        AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> GetSpawnables() const;
        bool GetSupportWGS() const;

    private:
        ROS2SpawnerComponentConfig m_config;
    };
} // namespace ROS2
