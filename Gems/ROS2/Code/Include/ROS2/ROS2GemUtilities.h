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
#ifdef ROS2_EDITOR
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#endif
namespace ROS2
{
    namespace Utils
    {
        /// Create component for a given entity in safe way.
        /// @param entityId entity that will own component
        /// @param componentType Uuid of component to create
        /// @return The created componentId if successful, otherwise returns an invalid id
        AZ::ComponentId CreateComponent(const AZ::EntityId entityId, const AZ::Uuid componentType);

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

    } // namespace Utils
} // namespace ROS2
