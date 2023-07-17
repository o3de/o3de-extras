/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Transform.h>
#include <ROS2/Spawner/SpawnerInfo.h>

namespace ROS2
{

    class ROS2SpawnPointComponentConfig final : public AZ::ComponentConfig
    {
    public:
        AZ_RTTI(ROS2SpawnPointComponentConfig, "{eb3e6937-0d1d-4a31-87d7-6d6663e3cf35}");

        ROS2SpawnPointComponentConfig() = default;
        ~ROS2SpawnPointComponentConfig() = default;

        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_name;
        AZStd::string m_info;

        AZ::EntityId m_editorEntityId;
    };

    //! SpawnPoint indicates a place which is suitable to spawn a robot.
    class ROS2SpawnPointComponentController
    {
    public:
        AZ_TYPE_INFO(ROS2SpawnPointComponentController, "{cd29d626-0205-4ca0-ac0f-5377e4fd84dd}");

        ROS2SpawnPointComponentController() = default;
        explicit ROS2SpawnPointComponentController(const ROS2SpawnPointComponentConfig& config);
        ~ROS2SpawnPointComponentController() = default;

        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // Controller component
        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const ROS2SpawnPointComponentConfig& config);
        const ROS2SpawnPointComponentConfig& GetConfiguration() const;
        //////////////////////////////////////////////////////////////////////////

        AZStd::pair<AZStd::string, SpawnPointInfo> GetInfo() const;

    private:
        ROS2SpawnPointComponentConfig m_config;
    };
} // namespace ROS2
