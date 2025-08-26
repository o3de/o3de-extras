/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>

namespace ROS2
{
    //! This method computes the namespace for a given entity based on the provided configuration.
    //! It considers the namespace strategy defined in the configuration and the entity's position in the hierarchy
    AZStd::string ComputeNamespace(const ROS2FrameConfiguration& configuration, AZ::EntityId entity);

    //! This method computes the namespace for vector of configurations and associated names.
    //! The configurations are expected to be ordered from the root to the leaf entity.
    AZStd::string ComputeNamespace(const AZStd::vector<AZStd::pair<AZStd::string, ROS2FrameConfiguration>>& configurations);

    //! Gets the namespaced name for the given entity, combining its namespace and frame name.
    //! for empty namespace, it returns just the name
    //! for non-empty namespace, it returns namespace/name
    AZStd::string GetNamespacedName(const AZStd::string& namespaceName, const AZStd::string& name);

    //! returns true if the entity has a ROS2FrameComponent or ROS2FrameEditorComponent
    //! @note to be used in Game and Editor contexts
    bool HasROS2FrameComponent(AZ::EntityId id);

    //! Returns all ancestor entities of the given entity that have a TransformComponent, including the entity itself.
    //! the root entity (e.g. level) is the last element in the vector
    //! @note to be used in Game and Editor contexts
    AZStd::vector<AZ::EntityId> GetAllAncestorTransformBus(const AZ::EntityId& id);

    //! Filters the given list of entity IDs and returns only those that have a ROS2FrameComponent or ROS2FrameEditorComponent.
    //! @note to be used in Game and Editor contexts
    AZStd::vector<AZ::EntityId> GetEntitiesWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors);

    //! Returns the first entity that has a ROS2FrameComponent or ROS2FrameEditorComponent.
    AZ::EntityId GetFirstEntityWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors);

    //! Returns the last entity that has a ROS2FrameComponent or ROS2FrameEditorComponent.
    AZ::EntityId GetLastEntityWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors);

} // namespace ROS2
