/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/ComponentBus.h"
#include "AzCore/Component/Entity.h"
#include "AzToolsFramework/ToolsComponents/GenericComponentWrapper.h"

namespace ROS2
{
    namespace Utils
    {
        /// Create component for a given entity in safe way.
        /// \param entityId entity that will own component
        /// \param componentType Uuid of component to create
        /// \return created componentId, if it fails, it returns invalid id
        AZ::ComponentId CreateComponent(const AZ::EntityId entityId, const AZ::Uuid componentType);

        /// Retrieve component from entity given by a pointer. It is a way to get game components and wrapped components.
        /// We should use that that we are not sure if we access e.g. ROS2FrameComponent in game mode or from Editor
        /// \param entity pointer to entity e.g. with GetEntity()
        /// \return pointer to component with type T

        template<class ComponentType>
        ComponentType* GetGameOrEditorComponent(const AZ::Entity* entity)
        {
            AZ_Assert(entity, "Called with empty entity");
            ComponentType* component = entity->FindComponent<ComponentType>();
            if (component)
            {
                return component;
            }
            // failed to get game object, let us retry as editor
            component = AzToolsFramework::FindWrappedComponentForEntity<ComponentType>(entity);
            AZ_Assert(entity, "Entity %s has no component of type", entity->GetId());
            // no component found
            return component;
        }

    } // namespace Utils
} // namespace ROS2