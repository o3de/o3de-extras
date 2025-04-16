/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnPointEditorComponent.h"
#include "Spawner/ROS2SpawnPointComponentController.h"
#include "Spawner/ROS2SpawnerEditorComponent.h"

namespace ROS2
{
    ROS2SpawnPointEditorComponent::ROS2SpawnPointEditorComponent(const ROS2SpawnPointComponentConfig& configuration)
        : ROS2SpawnPointEditorComponentBase(configuration)
    {
    }

    void ROS2SpawnPointEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnPointEditorComponentBase::Reflect(context);

        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);

        if (serializeContext)
        {
            serializeContext->Class<ROS2SpawnPointEditorComponent, ROS2SpawnPointEditorComponentBase>()->Version(1);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<ROS2SpawnPointEditorComponent>("ROS2 Spawn Point", "Spawn point for robots")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2SpawnPoint.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2SpawnPoint.svg")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2");
            }
        }
    }

    void ROS2SpawnPointEditorComponent::Activate()
    {
        ROS2SpawnPointEditorComponentBase::Activate();
    }

    void ROS2SpawnPointEditorComponent::Deactivate()
    {
        ROS2SpawnPointEditorComponentBase::Deactivate();
    }

    AZStd::pair<AZStd::string, SpawnPointInfo> ROS2SpawnPointEditorComponent::GetInfo() const
    {
        return m_controller.GetInfo();
    }
} // namespace ROS2
