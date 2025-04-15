/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ResetSimulationServiceHandler.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/std/optional.h>
#include <ROS2/Clock/ROS2Clock.h>
#include <ROS2/ROS2Bus.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <builtin_interfaces/msg/time.hpp>
#include <simulation_interfaces/msg/detail/result__struct.hpp>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<AZ::u8> ResetSimulationServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<AZ::u8>{ SimulationFeatures::SIMULATION_RESET,
                                             SimulationFeatures::SIMULATION_RESET_TIME,
                                             SimulationFeatures::SIMULATION_RESET_STATE,
                                             SimulationFeatures::SIMULATION_RESET_SPAWNED };
    }

    AZStd::optional<ResetSimulationServiceHandler::Response> ResetSimulationServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        if (request.scope == simulation_interfaces::srv::ResetSimulation::Request::SCOPE_STATE)
        {
            Response response;
            SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
                &SimulationInterfaces::SimulationEntityManagerRequests::ResetAllEntitiesToInitialState);
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            return response;
        }

        if (request.scope == simulation_interfaces::srv::ResetSimulation::Request::SCOPE_SPAWNED)
        {
            auto deletionCompletedCb = [this](const AZ::Outcome<void, SimulationInterfaces::FailedResult>& outcome)
            {
                Response response;
                if (outcome.IsSuccess())
                {
                    response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                }
                else
                {
                    const auto& failedResult = outcome.GetError();
                    response.result.result = aznumeric_cast<uint8_t>(failedResult.m_errorCode);
                    response.result.error_message = failedResult.m_errorString.c_str();
                }
                SendResponse(response);
            };
            SimulationInterfaces::SimulationEntityManagerRequestBus::Broadcast(
                &SimulationInterfaces::SimulationEntityManagerRequests::DeleteAllEntities, deletionCompletedCb);
            return AZStd::nullopt;
        }

        if (request.scope == Request::SCOPE_TIME)
        {
            auto* interface = ROS2::ROS2Interface::Get();
            AZ_Assert(interface, "ROS2Interface is not available");
            auto& clock = interface->GetSimulationClock();

            builtin_interfaces::msg::Time time;
            time.sec = 0;
            time.nanosec = 0;
            auto results = clock.AdjustTime(time);

            if (results.IsSuccess())
            {
                Response response;
                response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                return response;
            }
            else
            {
                Response response;
                response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
                const auto& errorMessage = results.GetError();
                response.result.error_message = std::string(errorMessage.c_str(), errorMessage.size());
                return response;
            }
        }

        if (request.scope == Request::SCOPE_ALL || request.scope == Request::SCOPE_DEFAULT)
        {
            // check if we are in GameLauncher
            AZ::ApplicationTypeQuery appType;
            AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
            if (appType.IsValid() && appType.IsEditor())
            {
                Response response;
                response.result.result = simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED;
                response.result.error_message = "Feature not supported in Editor.";
                return response;
            }

            auto levelReloadCompletion = [this]()
            {
                Response response;
                response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
                SendResponse(response);
            };
            SimulationInterfaces::SimulationManagerRequestBus::Broadcast(
                &SimulationInterfaces::SimulationManagerRequests::ReloadLevel, levelReloadCompletion);

            return AZStd::nullopt;
        }

        // no case matched, return response that request was invalid
        Response invalidResponse;
        invalidResponse.result.result = simulation_interfaces::msg::Result::RESULT_NOT_FOUND;
        invalidResponse.result.error_message = "Passed unknown scope";
        return invalidResponse;
    }
} // namespace ROS2SimulationInterfaces
