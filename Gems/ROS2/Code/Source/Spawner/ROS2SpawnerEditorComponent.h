/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Spawner/ROS2SpawnerComponent.h"
#include "Spawner/ROS2SpawnerComponentController.h"
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Spawner/SpawnerBus.h>

namespace ROS2
{
    using ROS2SpawnerEditorComponentBase = AzToolsFramework::Components::
        EditorComponentAdapter<ROS2SpawnerComponentController, ROS2SpawnerComponent, ROS2SpawnerComponentConfig>;

    class ROS2SpawnerEditorComponent
        : public ROS2SpawnerEditorComponentBase
        , public SpawnerRequestsBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(
            ROS2SpawnerEditorComponent, "{5950AC6B-75F3-4E0F-BA5C-17C877013710}", AzToolsFramework::Components::EditorComponentBase);
        ROS2SpawnerEditorComponent() = default;
        explicit ROS2SpawnerEditorComponent(const ROS2SpawnerComponentConfig& configuration);
        ~ROS2SpawnerEditorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // ROS2SpawnerEditorComponentBase interface overrides.
        void Activate() override;
        void Deactivate() override;
        bool ShouldActivateController() const override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // SpawnerRequestsBus::Handler overrides.
        const AZ::Transform& GetDefaultSpawnPose() const override;
        AZStd::unordered_map<AZStd::string, SpawnPointInfo> GetAllSpawnPointInfos() const override;
        //////////////////////////////////////////////////////////////////////////

        AZStd::unordered_map<AZStd::string, SpawnPointInfo> GetSpawnPoints() const;
    };
} // namespace ROS2
