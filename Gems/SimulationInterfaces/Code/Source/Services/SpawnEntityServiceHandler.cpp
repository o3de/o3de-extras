/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SpawnEntityServiceHandler.h"
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> SpawnEntityServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::SPAWNING };
    }

    AZStd::optional<SpawnEntityServiceHandler::Response> SpawnEntityServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        const AZStd::string_view name{ request.name.c_str(), request.name.size() };
        const AZStd::string_view uri{ request.uri.c_str(), request.uri.size() };
        const AZStd::string_view entityNamespace{ request.entity_namespace.c_str(), request.entity_namespace.size() };

        // Validate entity name
        if (!name.empty() && !ValidateEntityName(name))
        {
            Response response;
            response.result.result = simulation_interfaces::srv::SpawnEntity::Response::NAME_INVALID;
            response.result.error_message = "Invalid entity name. Entity names can only contain alphanumeric characters and underscores.";
            SendResponse(response);
            return AZStd::nullopt;
        }

        // Validate namespace name
        if (!entityNamespace.empty() && !ValidateNamespaceName(entityNamespace))
        {
            Response response;
            response.result.result = simulation_interfaces::srv::SpawnEntity::Response::NAMESPACE_INVALID;
            response.result.error_message =
                "Invalid entity namespace. Entity namespaces can only contain alphanumeric characters and forward slashes.";
            SendResponse(response);
            return AZStd::nullopt;
        }

        const AZ::Transform initialPose = ROS2::ROS2Conversions::FromROS2Pose(request.initial_pose.pose);
        SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationEntityManagerRequests::SpawnEntity,
            name,
            uri,
            entityNamespace,
            initialPose,
            request.allow_renaming,
            [this](const AZ::Outcome<AZStd::string, SimulationInterfaces::FailedResult>& outcome)
            {
                Response response;
                if (outcome.IsSuccess())
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                    response.entity_name = outcome.GetValue().c_str();
                }
                else
                {
                    const auto& failedResult = outcome.GetError();
                    response.result.result = failedResult.m_errorCode;
                    response.result.error_message = failedResult.m_errorString.c_str();
                }
                SendResponse(response);
            });
        return AZStd::nullopt;
    }

    bool SpawnEntityServiceHandler::ValidateEntityName(const AZStd::string& entityName)
    {
        const AZStd::regex entityRegex{ R"(^[a-zA-Z0-9_]+$)" }; // Entity names can only contain alphanumeric characters and underscores
        return AZStd::regex_match(entityName, entityRegex);
    }

    bool SpawnEntityServiceHandler::ValidateNamespaceName(const AZStd::string& namespaceName)
    {
        const AZStd::regex namespaceRegex{
            R"(^[a-zA-Z0-9_/]+$)"
        }; // Namespace names can only contain alphanumeric characters and underscores and forward slashes
        return AZStd::regex_match(namespaceName, namespaceRegex);
    }

} // namespace ROS2SimulationInterfaces
