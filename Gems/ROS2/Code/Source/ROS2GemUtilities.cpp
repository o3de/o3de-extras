/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2GemUtilities.h"
#include <AzCore/std/string/regex.h>
#include <AzToolsFramework/API/EntityCompositionRequestBus.h>

namespace ROS2
{

    AZ::ComponentId Utils::CreateComponent(const AZ::EntityId entityId, const AZ::Uuid componentType)
    {
        AZ::ComponentTypeList componentsToAdd{ componentType };
        AZStd::vector<AZ::EntityId> entityIds{ entityId };
        AzToolsFramework::EntityCompositionRequests::AddComponentsOutcome addComponentsOutcome = AZ::Failure(AZStd::string());
        AzToolsFramework::EntityCompositionRequestBus::BroadcastResult(
            addComponentsOutcome, &AzToolsFramework::EntityCompositionRequests::AddComponentsToEntities, entityIds, componentsToAdd);
        if (!addComponentsOutcome.IsSuccess())
        {
            AZ_Warning(
                "URDF importer",
                false,
                "Failed to create component %s, entity %s : %s",
                componentType.ToString<AZStd::string>().c_str(),
                entityId.ToString().c_str(),
                addComponentsOutcome.GetError().c_str());
        }
        const auto& added = addComponentsOutcome.GetValue().at(entityId).m_componentsAdded;
        if (added.size())
        {
            return added.front()->GetId();
        }
        return AZ::InvalidComponentId;
    }

} // namespace ROS2
