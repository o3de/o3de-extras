/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControlMaker.h"

#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <ROS2RobotImporter/ROS2RobotImporterBus.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

namespace ROS2RobotImporter
{
    RobotControlMaker::ControlHookCallOutcome RobotControlMaker::AddPlugin(
        AZ::EntityId entityId, const sdf::Plugin& plugin, const sdf::Model& model, const SDFormat::CreatedEntitiesMap& createdEntities)
    {
        SDFormat::ModelPluginImporterHooksStorage pluginHooks;
        RobotImporterRequestBus::BroadcastResult(pluginHooks, &RobotImporterRequest::GetModelPluginHooks);
        for (const auto& hook : pluginHooks)
        {
            const AZStd::string query(plugin.Filename().c_str());
            if (hook.m_pluginNames.contains(query))
            {
                AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
                const auto outcome = hook.m_sdfPluginToComponentCallback(*entity, plugin, model, createdEntities);

                if (outcome.IsSuccess())
                {
                    const auto pluginElement = plugin.Element();
                    const auto& unsupportedPluginParams =
                        Utils::SDFormat::GetUnsupportedParams(pluginElement, hook.m_supportedPluginParams);
                    AZStd::string status;
                    if (unsupportedPluginParams.empty())
                    {
                        status = AZStd::string::format(
                            "%s (filename %s) created successfully", plugin.Name().c_str(), plugin.Filename().c_str());
                    }
                    else
                    {
                        status = AZStd::string::format(
                            "%s (filename %s) created, %lu parameters not parsed: ",
                            plugin.Name().c_str(),
                            plugin.Filename().c_str(),
                            unsupportedPluginParams.size());
                        for (const auto& up : unsupportedPluginParams)
                        {
                            status.append("\n\t - " + up);
                        }
                    }
                    m_status.emplace(AZStd::move(status));

                    return AZ::Success();
                }
                else
                {
                    const auto message = AZStd::string::format(
                        "%s (filename %s) not created: %s", plugin.Name().c_str(), plugin.Filename().c_str(), outcome.GetError().c_str());
                    m_status.emplace(message);
                    return AZ::Failure(message);
                }
            }
        }

        const auto message =
            AZStd::string::format("%s (filename %s) not created: cannot find the hook", plugin.Name().c_str(), plugin.Filename().c_str());
        m_status.emplace(message);
        return AZ::Failure(message);
    }

    void RobotControlMaker::AddControlPlugins(
        const sdf::Model& model, AZ::EntityId entityId, const SDFormat::CreatedEntitiesMap& createdEntities)
    {
        const auto plugins = model.Plugins();
        for (const auto& plugin : plugins)
        {
            const auto outcome = AddPlugin(entityId, plugin, model, createdEntities);
            AZ_Warning("RobotControlMaker", outcome.IsSuccess(), outcome.GetError().c_str());
        }
    }

    const AZStd::set<AZStd::string>& RobotControlMaker::GetStatusMessages() const
    {
        return m_status;
    }
} // namespace ROS2RobotImporter
