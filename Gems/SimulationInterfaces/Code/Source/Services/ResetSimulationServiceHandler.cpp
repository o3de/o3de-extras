/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ResetSimulationServiceHandler.h"
#include "Services/ROS2ServiceBase.h"
#include "SimulationInterfaces/ROS2SimulationInterfacesRequestBus.h"
#include "SimulationInterfaces/Result.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/optional.h>
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/ROS2Bus.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <builtin_interfaces/msg/time.hpp>
#include <simulation_interfaces/msg/result.hpp>

namespace ROS2SimulationInterfaces
{
    AZStd::unordered_set<SimulationFeatureType> ResetSimulationServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::SIMULATION_RESET,
                                                            SimulationFeatures::SIMULATION_RESET_TIME,
                                                            SimulationFeatures::SIMULATION_RESET_STATE,
                                                            SimulationFeatures::SIMULATION_RESET_SPAWNED };
    }

    AZStd::optional<ResetSimulationServiceHandler::Response> ResetSimulationServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        if (request.scope == Request::SCOPE_STATE)
        {
            Response response;
            AZ::Outcome<void, SimulationInterfaces::FailedResult> resetingOutcome;
            SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
                resetingOutcome, &SimulationInterfaces::SimulationEntityManagerRequests::ResetAllEntitiesToInitialState);
            if (resetingOutcome.IsSuccess())
            {
                response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            }
            else
            {
                response.result.result = resetingOutcome.GetError().m_errorCode;
                response.result.error_message = resetingOutcome.GetError().m_errorString.c_str();
            }
            return response;
        }

        if (request.scope == Request::SCOPE_SPAWNED)
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
                    response.result.result = failedResult.m_errorCode;
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
            auto* interface = ROS2::ROS2ClockInterface::Get();
            AZ_Assert(interface, "ROS2ClockInterface is not available");

            builtin_interfaces::msg::Time time;
            time.sec = 0;
            time.nanosec = 0;
            auto results = interface->AdjustTime(time);

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
            AZ::Outcome<void, SimulationInterfaces::FailedResult> restartPossible;
            SimulationInterfaces::SimulationManagerRequestBus::BroadcastResult(
                restartPossible, &SimulationInterfaces::SimulationManagerRequests::RestartSimulation, levelReloadCompletion);
            if (!restartPossible.IsSuccess())
            {
                Response invalidResponse;
                invalidResponse.result.result = restartPossible.GetError().m_errorCode;
                invalidResponse.result.error_message = restartPossible.GetError().m_errorString.c_str();
                return invalidResponse;
            }

            return AZStd::nullopt;
        }

        // no case matched, return response that request was invalid
        Response invalidResponse;
        invalidResponse.result.result = simulation_interfaces::msg::Result::RESULT_NOT_FOUND;
        invalidResponse.result.error_message = "Passed unknown scope";
        return invalidResponse;
    }
} // namespace ROS2SimulationInterfaces
