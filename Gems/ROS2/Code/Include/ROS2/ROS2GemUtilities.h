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
#include <AzCore/Component/EntityUtils.h>
#ifdef ROS2_EDITOR
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#endif

namespace ROS2
{
    namespace Utils
    {
        //! Checks whether the entity has a component of the given type
        //! @param entity pointer to entity
        //! @param typeId type of the component
        //! @returns true if entity has component with given type
        inline bool HasComponentOfType(const AZ::Entity* entity, const AZ::Uuid typeId)
        {
            auto components = AZ::EntityUtils::FindDerivedComponents(entity, typeId);
            return !components.empty();
        }

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
