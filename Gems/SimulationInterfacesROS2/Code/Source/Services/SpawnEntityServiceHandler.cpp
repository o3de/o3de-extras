/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SpawnEntityServiceHandler.h"
#include "Services/ROS2HandlerBaseClass.h"
#include <AzCore/std/optional.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace SimulationInterfacesROS2
{

    AZStd::unordered_set<AZ::u8> SpawnEntityServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::SPAWNING };
    }

    AZStd::optional<SpawnEntityServiceHandler::Response> SpawnEntityServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        const AZStd::string_view name{ request.name.c_str(), request.name.size() };
        const AZStd::string_view uri{ request.uri.c_str(), request.uri.size() };
        const AZStd::string_view entityNamespace{ request.entity_namespace.c_str(), request.entity_namespace.size() };
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
                    response.result.result = aznumeric_cast<uint8_t>(failedResult.error_code);
                    response.result.error_message = failedResult.error_string.c_str();
                }
                SendResponse(response);
            });
        return AZStd::nullopt;
    }

} // namespace SimulationInterfacesROS2
