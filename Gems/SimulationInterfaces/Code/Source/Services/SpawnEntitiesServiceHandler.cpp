/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SpawnEntitiesServiceHandler.h"
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/TF/TransformInterface.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/RegistryUtils.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> SpawnEntitiesServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::SPAWNING_ENTITIES };
    }

    AZStd::optional<SpawnEntitiesServiceHandler::Response> SpawnEntitiesServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        const builtin_interfaces::msg::Time zeroTime = builtin_interfaces::msg::Time();
        const auto simulatorFrameId = RegistryUtilities::GetSimulatorROS2Frame();

        Response response;
        response.results.resize(request.spawn_requests.size());

        AZStd::vector<SimulationInterfaces::SpawningEntity> spawningEntities;
        spawningEntities.reserve(request.spawn_requests.size());
        AZStd::vector<size_t> spawnedRequestsIndices;
        spawnedRequestsIndices.reserve(request.spawn_requests.size());

        bool hasFailures = false;
        for (size_t requestIdx = 0; requestIdx < request.spawn_requests.size(); ++requestIdx)
        {
            const auto& spawnRequest = request.spawn_requests[requestIdx];
            auto& spawnResult = response.results[requestIdx];

            const AZStd::string_view name{ spawnRequest.name.c_str(), spawnRequest.name.size() };
            const AZStd::string_view uri{ spawnRequest.entity_resource.uri.c_str(), spawnRequest.entity_resource.uri.size() };
            const AZStd::string_view entityNamespace{ spawnRequest.entity_namespace.c_str(), spawnRequest.entity_namespace.size() };
            const AZStd::string_view messageFrameId{ spawnRequest.initial_pose.header.frame_id.c_str(),
                                                     spawnRequest.initial_pose.header.frame_id.size() };

            if (!name.empty() && !ValidateEntityName(name))
            {
                spawnResult.result.result = simulation_interfaces::msg::SpawnResult::NAME_INVALID;
                spawnResult.result.error_message =
                    "Invalid entity name. Entity names can only contain alphanumeric characters and underscores.";
                hasFailures = true;
                continue;
            }

            if (!entityNamespace.empty() && !ValidateNamespaceName(entityNamespace))
            {
                spawnResult.result.result = simulation_interfaces::msg::SpawnResult::NAMESPACE_INVALID;
                spawnResult.result.error_message =
                    "Invalid entity namespace. Entity namespaces can only contain alphanumeric characters and forward slashes.";
                hasFailures = true;
                continue;
            }

            AZ::Transform transformOffset = AZ::Transform::CreateIdentity();
            if (!messageFrameId.empty() && simulatorFrameId != messageFrameId)
            {
                auto transformInterface = ROS2::TFInterface::Get();
                AZ_Assert(transformInterface, "TFInterface is not available, cannot set entity state without transform offset.");
                const auto transformOutcome = transformInterface->GetTransform(simulatorFrameId, messageFrameId, zeroTime);

                if (transformOutcome.IsSuccess())
                {
                    transformOffset = transformOutcome.GetValue();
                }
                else
                {
                    spawnResult.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
                    spawnResult.result.error_message = transformOutcome.GetError().c_str();
                    hasFailures = true;
                    continue;
                }
            }

            SimulationInterfaces::SpawningEntity spawningEntity;
            spawningEntity.name = AZStd::string(name);
            spawningEntity.uri = AZStd::string(uri);
            spawningEntity.entityNamespace = AZStd::string(entityNamespace);
            spawningEntity.initialPose = transformOffset * ROS2::ROS2Conversions::FromROS2Pose(spawnRequest.initial_pose.pose);
            spawningEntity.allowRename = spawnRequest.allow_renaming;
            spawningEntity.preinsertionCb =
                [](const AZ::Outcome<AzFramework::SpawnableEntityContainerView, SimulationInterfaces::FailedResult>&)
            {
            };
            spawningEntity.completedCb = [](const AZ::Outcome<AZStd::string, SimulationInterfaces::FailedResult>&)
            {
            };

            spawningEntities.push_back(AZStd::move(spawningEntity));
            spawnedRequestsIndices.push_back(requestIdx);
        }

        if (spawningEntities.empty())
        {
            response.result.result = hasFailures ? simulation_interfaces::srv::SpawnEntities::Response::ENTITIES_SPAWN_FAILED
                                                 : simulation_interfaces::msg::Result::RESULT_OK;
            if (hasFailures)
            {
                response.result.error_message = "One or more entity spawn requests failed.";
            }
            SendResponse(response);
            return AZStd::nullopt;
        }

        SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
            &SimulationInterfaces::SimulationEntityManagerRequests::SpawnEntities,
            spawningEntities,
            [this, response, spawnedRequestsIndices, hasFailures](const SimulationInterfaces::BatchSpawnResult& batchResult) mutable
            {
                bool hasBatchFailures = hasFailures;

                for (size_t spawnedIdx = 0; spawnedIdx < batchResult.m_spawnResults.size() && spawnedIdx < spawnedRequestsIndices.size();
                     ++spawnedIdx)
                {
                    const size_t requestIdx = spawnedRequestsIndices[spawnedIdx];
                    auto& spawnResult = response.results[requestIdx];
                    const auto& outcome = batchResult.m_spawnResults[spawnedIdx];

                    if (outcome.IsSuccess())
                    {
                        spawnResult.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                        spawnResult.entity_name = outcome.GetValue().c_str();
                    }
                    else
                    {
                        const auto& failedResult = outcome.GetError();
                        spawnResult.result.result = failedResult.m_errorCode;
                        spawnResult.result.error_message = failedResult.m_errorString.c_str();
                        hasBatchFailures = true;
                    }
                }

                response.result.result = hasBatchFailures ? simulation_interfaces::srv::SpawnEntities::Response::ENTITIES_SPAWN_FAILED
                                                          : simulation_interfaces::msg::Result::RESULT_OK;
                if (hasBatchFailures)
                {
                    response.result.error_message = "One or more entity spawn requests failed.";
                }

                SendResponse(response);
            });

        return AZStd::nullopt;
    }

    bool SpawnEntitiesServiceHandler::ValidateEntityName(const AZStd::string& entityName)
    {
        const AZStd::regex entityRegex{ R"(^[a-zA-Z0-9_]+$)" }; // Entity names can only contain alphanumeric characters and underscores
        return AZStd::regex_match(entityName, entityRegex);
    }

    bool SpawnEntitiesServiceHandler::ValidateNamespaceName(const AZStd::string& namespaceName)
    {
        const AZStd::regex namespaceRegex{
            R"(^[a-zA-Z0-9_/]+$)"
        }; // Namespace names can only contain alphanumeric characters and underscores and forward slashes
        return AZStd::regex_match(namespaceName, namespaceRegex);
    }

} // namespace ROS2SimulationInterfaces
