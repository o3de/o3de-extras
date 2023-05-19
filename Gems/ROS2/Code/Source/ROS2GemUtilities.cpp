/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/std/string/regex.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    Multiplayer::NetBindComponent* Utils::GetEntityOrAncestorNetBind(const AZ::Entity* entity)
    {
        AZ_Printf("Utils::GetEntityOrAncestorNetBind", "-> Entity %s\n", entity->GetName().c_str());
        
        if(auto* component = GetGameOrEditorComponent<Multiplayer::NetBindComponent>(entity))
        {
            return component; // Found it!
        }

        // Entity has no NetBindComponent. Let's check its ancestor
        auto* entityTransformInterface = GetGameOrEditorComponent<AzFramework::TransformComponent>(entity);
        if (!entityTransformInterface)
        {
            AZ_Error("GetEntityOrAncestorNetBind", false, "Invalid transform interface!");
            return nullptr;
        }

        AZ::EntityId parentEntityId = entityTransformInterface->GetParentId();
        if (!parentEntityId.IsValid())
        { // We have reached the top level, there is no parent entity
            return nullptr;
        }

        AZ::Entity* parentEntity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, parentEntityId);
        AZ_Assert(parentEntity, "No parent entity id : %s", parentEntityId.ToString().c_str());

        return GetEntityOrAncestorNetBind(parentEntity);
    }

    bool Utils::IsAutonomousOrNonMultiplayer(const AZ::Entity* entity) 
    {
#ifdef ROS2_EDITOR
        return true; // Always enable everything within the editor
#endif
        bool responsible = true;
        if(Multiplayer::NetBindComponent* nbc = GetEntityOrAncestorNetBind(entity))
        {
            // return nbc->IsNetEntityRoleAutonomous();
            responsible = nbc->IsNetEntityRoleAutonomous();
        }
        // return true; // Non-multiplayer: No NetBindComponent, so no multiplayer entity in the hierarchy  

        AZ_Printf("Utils::IsAutonomousOrNonMultiplayer", "Entity %s - %s\n", entity->GetName().c_str(), responsible ? "true" : "false");
        // return responsible;  
#if AZ_TRAIT_SERVER
        AZ_Printf("Utils::IsAutonomousOrNonMultiplayer", "### I AM A SERVER ###\n");
        return false;
#else
        AZ_Printf("Utils::IsAutonomousOrNonMultiplayer", "### I AM A CLIENT ###\n");
        return true;
#endif
    }

} // namespace ROS2
