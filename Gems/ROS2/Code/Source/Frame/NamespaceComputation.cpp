/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamespaceComputation.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2
{
    namespace
    {
        inline constexpr const char* DefaultGlobalFrameName = "odom";
        inline constexpr const char* DefaultGlobalFrameNameConfigurationKey = "/O3DE/ROS2/GlobalFrameName";
        constexpr char FramePathSeparator = '/';

        //! Helper to get the name of an entity from its EntityId.
        AZStd::string GetName(AZ::EntityId id)
        {
            AZStd::string name;
            AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, id);
            return name;
        }

        //! Helper to concatenate parts of a path with '/' separator.
        AZStd::string ConcatenatePath(const AZStd::vector<AZStd::string>& path)
        {
            AZStd::string result;
            for (const auto& part : path)
            {
                if (!result.empty())
                {
                    result += FramePathSeparator;
                }
                result += part;
            }
            return result;
        }

        //! Helper to retrieve the ROS2FrameConfiguration from the entity with the given ID, if it has a ROS2FrameComponent or
        //! ROS2FrameEditorComponent.
        AZStd::optional<ROS2FrameConfiguration> GetConfigurationFromComponent(AZ::EntityId id)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
            AZ_Assert(entity, "No entity for id : %s", id.ToString().c_str());
            if (!entity)
            {
                return AZStd::nullopt;
            }
            const auto* componentEditor = entity->FindComponent(AZ::Uuid(ROS2FrameEditorComponentTypeId));
            if (componentEditor)
            {
                const auto* interface = dynamic_cast<const ROSFrameInterface*>(componentEditor);
                if (interface)
                {
                    return interface->GetConfiguration();
                }
            }
            const auto* componentGame = entity->FindComponent(AZ::Uuid(ROS2FrameComponentTypeId));
            if (componentGame)
            {
                const auto* interface = dynamic_cast<const ROSFrameInterface*>(componentGame);
                if (interface)
                {
                    return interface->GetConfiguration();
                }
            }
            return AZStd::nullopt;
        }

        //! Helper function to recursively traverse the transform hierarchy and collect ancestor entity IDs.
        //! Uses AZ::TransformInterface from O3DE to get parent entity IDs.
        void TraverseTransforms(AZ::EntityId id, AZStd::vector<AZ::EntityId>& predecessors)
        {
            predecessors.push_back(id);
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
            AZ_Assert(entity, "No entity for id : %s", id.ToString().c_str());
            if (!entity)
            {
                return;
            }
            auto* transformInterface = entity->GetTransform();
            AZ_Assert(transformInterface, "No transform for id : %s", id.ToString().c_str());
            if (!transformInterface)
            {
                return;
            }
            AZ::EntityId parentId = transformInterface->GetParentId();
            if (!parentId.IsValid())
            {
                return;
            }
            TraverseTransforms(parentId, predecessors);
        }
    } // namespace

    bool HasROS2FrameComponent(AZ::EntityId id)
    {
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, id);
        if (!entity)
        {
            return false;
        }
        return (
            entity->FindComponent(AZ::Uuid(ROS2FrameEditorComponentTypeId)) || entity->FindComponent(AZ::Uuid(ROS2FrameComponentTypeId)));
    }

    AZStd::vector<AZ::EntityId> GetAllAncestorTransformBus(const AZ::EntityId& id)
    {
        AZStd::vector<AZ::EntityId> predecessors;
        TraverseTransforms(id, predecessors);
        return predecessors;
    }

    AZStd::vector<AZ::EntityId> GetEntitiesWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& unfiletered)
    {
        AZStd::vector<AZ::EntityId> filtered;
        for (auto it = unfiletered.begin(); it != unfiletered.end(); ++it)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, *it);
            if (!entity)
            {
                continue;
            }
            if (HasROS2FrameComponent(*it))
            {
                filtered.push_back(*it);
            }
        }
        return filtered;
    }

    AZ::EntityId GetFirstEntityWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors)
    {
        for (auto it = predecessors.rbegin(); it != predecessors.rend(); ++it)
        {
            if (HasROS2FrameComponent(*it))
            {
                return *it;
            }
        }
        return AZ::EntityId(AZ::EntityId::InvalidEntityId);
    }

    AZ::EntityId GetLastEntityWithROS2FrameComponent(const AZStd::vector<AZ::EntityId>& predecessors)
    {
        for (auto it = predecessors.begin(); it != predecessors.end(); ++it)
        {
            if (HasROS2FrameComponent(*it))
            {
                return *it;
            }
        }
        return AZ::EntityId(AZ::EntityId::InvalidEntityId);
    }

    AZStd::string GetNamespacedName(const AZStd::string& namespaceName, const AZStd::string& name)
    {
        if (namespaceName.empty())
        {
            return name;
        }
        return namespaceName + "/" + name;
    }

    AZStd::string ComputeNamespace(const AZStd::vector<AZStd::pair<AZStd::string, ROS2FrameConfiguration>>& configurations)
    {
        if (configurations.empty())
        {
            return "";
        }
        AZStd::vector<AZStd::string> resolvedNames;

        for (auto it = configurations.rbegin(); it != configurations.rend(); ++it)
        {
            const auto& ancestorName = it->first;
            const auto& ancestorConfig = it->second;
            const auto ancestorStrategy = ancestorConfig.m_namespaceConfiguration.m_namespaceStrategy;
            if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::FromEntityName)
            {
                resolvedNames.emplace_back(ancestorName);
            }
            else if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::Custom)
            {
                resolvedNames.emplace_back(ancestorConfig.m_namespaceConfiguration.m_customNamespace);
            }
            else if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::Empty)
            {
                void(); // do nothing
            }
            else if (ancestorStrategy == NamespaceConfiguration::NamespaceStrategy::Default)
            {
                if (it == configurations.rbegin())
                {
                    resolvedNames.emplace_back(ancestorName);
                }
            }
        }
        return ConcatenatePath(resolvedNames);
    }

    AZStd::string ComputeNamespace(AZ::EntityId entity)
    {
        // Compute namespace based on ancestor frames.
        const AZStd::vector<AZ::EntityId> predecessors = GetAllAncestorTransformBus(entity);
        const AZStd::vector<AZ::EntityId> predecessorsWithRos2Frame = GetEntitiesWithROS2FrameComponent(predecessors);
        AZStd::vector<AZStd::pair<AZStd::string, ROS2FrameConfiguration>> configurations;
        for (const auto& predecessorId : predecessorsWithRos2Frame)
        {
            if (auto config = GetConfigurationFromComponent(predecessorId); config.has_value())
            {
                const auto name = GetName(predecessorId);
                configurations.emplace_back(name, config.value());
            }
        }
        return ComputeNamespace(configurations);
    }

    AZStd::string GetGlobalFrameIDFromRegistry()
    {
        // Get odometry frame, from settings registry
        AZStd::string odometryFrame;
        auto* registry = AZ::SettingsRegistry::Get();
        AZ_Error("ROS2FrameComponent", registry, "No settings registry found, using default odometry frame name");
        if (registry)
        {
            if (!registry->Get(odometryFrame, DefaultGlobalFrameNameConfigurationKey))
            {
                odometryFrame = DefaultGlobalFrameName;
            }
        }

       return odometryFrame;
    }

} // namespace ROS2
