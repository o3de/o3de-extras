/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/Entity.h>
#include <AzFramework/Components/TransformComponent.h>
#ifdef ROS2_EDITOR
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#endif
#include <Multiplayer/Components/NetBindComponent.h>

namespace ROS2
{
    namespace Utils
    {
        /// Retrieve component from entity given by a pointer. It is a way to get game components and wrapped components.
        /// We should use that that we are not sure if we access eg ROS2FrameComponent in game mode or from Editor
        /// @param entity pointer to entity eg with GetEntity()
        /// @return pointer to component with type T

        template<class ComponentType>
        ComponentType* GetGameOrEditorComponent(const AZ::Entity* entity)
        {
            AZ_Assert(entity, "Called with empty entity");
            ComponentType* component = entity->FindComponent<ComponentType>();
            if (component)
            {
                return component;
            }
#ifdef ROS2_EDITOR
            // failed to get game object, let us retry as editor
            component = AzToolsFramework::FindWrappedComponentForEntity<ComponentType>(entity);
#endif
            return component;
        }

        /// Get the first found NetBindComponent* in the entity's transform hierarchy (or nullptr)
        /// @param entity 
        /// @return The first found NetBindComponent* or nullptr if none was found up the the scene root
        Multiplayer::NetBindComponent* GetEntityOrAncestorNetBind(const AZ::Entity* entity);

        /// Checks if the simulation of the provided entity is controlled by the current process.
        /// Used by ROS2 components in multiplayer mode to determine if they should activate themselves or not.
        /// Thus, ensuring they are only simulated on a single client.
        /// @param entity 
        /// @return `False` iff `entity` is a multiplayer entity (has a `NetBindComponent` in its transform ancestry) 
        ///         and does NOT have `Autonomous` role. 
        ///         `True` otherwise.
        /// @note   Always returns `true` for Editor builds to ensure components can be configured reliably.
        bool IsAutonomousOrNonMultiplayer(const AZ::Entity* entity);

    } // namespace Utils
} // namespace ROS2
