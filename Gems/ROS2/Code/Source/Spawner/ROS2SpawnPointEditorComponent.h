/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Spawner/ROS2SpawnPointComponent.h"
#include "Spawner/ROS2SpawnPointComponentController.h"
#include "Spawner/ROS2SpawnerEditorComponent.h"
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

namespace ROS2
{
    using ROS2SpawnPointEditorComponentBase = AzToolsFramework::Components::
        EditorComponentAdapter<ROS2SpawnPointComponentController, ROS2SpawnPointComponent, ROS2SpawnPointComponentConfig>;

    class ROS2SpawnPointEditorComponent : public ROS2SpawnPointEditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(
            ROS2SpawnPointEditorComponent, "{2AE1CAAE-B300-49FD-8F6D-F7AAABED1EC3}", AzToolsFramework::Components::EditorComponentBase);

        ROS2SpawnPointEditorComponent() = default;
        ROS2SpawnPointEditorComponent(const ROS2SpawnPointComponentConfig& config);
        ~ROS2SpawnPointEditorComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // ROS2SpawnPointEditorComponentBase overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        AZStd::pair<AZStd::string, SpawnPointInfo> GetInfo() const;
    };
} // namespace ROS2
