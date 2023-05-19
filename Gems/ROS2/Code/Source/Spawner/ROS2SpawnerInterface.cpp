/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerInterface.h"
#include "ROS2SpawnPointComponent.h"
#include "ROS2SpawnerComponent.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/utils.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <Entity/EditorEntityInfoBus.h>

namespace ROS2
{
    const AZ::Transform& ROS2SpawnerInterface::GetDefaultSpawnPose() const
    {
        return m_defaultSpawnPose;
    };

    const AZStd::vector<AZStd::pair<AZStd::string, AZStd::shared_ptr<AZ::Transform>>> ROS2SpawnerInterface::GetAllSpawnPoints() const
    {
        AZStd::vector<AZStd::pair<AZStd::string, AZStd::shared_ptr<AZ::Transform>>> allSpawnPoints;

        AZ::EntityId levelEntityId;
        AzToolsFramework::ToolsApplicationRequestBus::BroadcastResult(
            levelEntityId, &AzToolsFramework::ToolsApplicationRequests::GetCurrentLevelEntityId);

        AZ::Entity* levelEntity{ nullptr };
        AZ::ComponentApplicationBus::BroadcastResult(levelEntity, &AZ::ComponentApplicationRequests::FindEntity, levelEntityId);

        if (!levelEntity)
        {
            AZ_Warning("ROS2SpawnerInterface", false, "Level entity is null");
            return allSpawnPoints;
        }
        else
        {
            AZ_TracePrintf("ROS2SpawnerInterface", "Level entity found");
        }

        AzToolsFramework::EntityIdList allLevelEntities;
        GetAllEntityDescendants(levelEntityId, allLevelEntities);

        const auto ROS2SpawnPointTypeId = AZ::AzTypeInfo<ROS2SpawnPointComponent>::Uuid();

        allSpawnPoints.push_back(AZStd::make_pair("Default position", AZStd::make_shared<AZ::Transform>(m_defaultSpawnPose)));

        for (const AZ::EntityId& entityId : allLevelEntities)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationBus::Events::FindEntity, entityId);
            AZ_Assert(entity, "No child entity %s", entityId.ToString().c_str());

            const auto allComponents = entity->GetComponents();

            for (const AZ::Component* component : allComponents)
            {
                if (component->GetUnderlyingComponentType() == ROS2SpawnPointTypeId)
                {
                    auto name = entity->GetName();
                    allSpawnPoints.push_back(
                        AZStd::make_pair(name, AZStd::make_shared<AZ::Transform>(entity->GetTransform()->GetWorldTM())));
                }
            }
        }

        return allSpawnPoints;
    }

    void ROS2SpawnerInterface::GetAllEntityDescendants(AZ::EntityId entityId, AzToolsFramework::EntityIdList& entityList) const
    {
        AzToolsFramework::EntityIdList children;
        AzToolsFramework::EditorEntityInfoRequestBus::EventResult(
            children, entityId, &AzToolsFramework::EditorEntityInfoRequestBus::Events::GetChildren);

        for (const AZ::EntityId& childId : children)
        {
            entityList.push_back(childId);
            GetAllEntityDescendants(childId, entityList);
        }
    }

} // namespace ROS2