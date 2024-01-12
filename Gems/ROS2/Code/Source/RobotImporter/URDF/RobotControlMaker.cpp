/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControlMaker.h"

#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <ROS2/RobotImporter/RobotImporterBus.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

namespace ROS2
{
    bool AddPlugin(
        AZ::EntityId entityId, const sdf::Plugin& plugin, const sdf::Model& model, const SDFormat::CreatedEntitiesMap& createdEntities)
    {
        SDFormat::ModelPluginImporterHooksStorage pluginHooks;
        ROS2::RobotImporterRequestBus::BroadcastResult(pluginHooks, &ROS2::RobotImporterRequest::GetModelPluginHooks);
        for (const auto& hook : pluginHooks)
        {
            const AZStd::string query(plugin.Filename().c_str());
            if (hook.m_pluginNames.contains(query))
            {
                AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                hook.m_sdfPluginToComponentCallback(*entity, plugin, model, createdEntities);
                return true;
            }
        }

        return false;
    }

    void RobotControlMaker::AddControlPlugins(
        const sdf::Model& model, AZ::EntityId entityId, const SDFormat::CreatedEntitiesMap& createdEntities) const
    {
        const auto plugins = model.Plugins();
        for (const auto& plugin : plugins)
        {
            const bool success = AddPlugin(entityId, plugin, model, createdEntities);
            AZ_Warning("RobotControlMaker", success, "Cannot find a model plugin hook for plugin %s", plugin.Name().c_str());
        }
    }
} // namespace ROS2
