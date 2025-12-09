/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetEntitiesStatesServiceHandler.h"
#include "ROS2/ROS2Bus.h"
#include <Clients/CommonUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/RegistryUtils.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{
    AZStd::unordered_set<SimulationFeatureType> GetEntitiesStatesServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::ENTITY_TAGS,
                                                            SimulationFeatures::ENTITY_BOUNDS_BOX,
                                                            SimulationFeatures::ENTITY_BOUNDS_CONVEX,
                                                            SimulationFeatures::ENTITY_CATEGORIES,
                                                            SimulationFeatures::ENTITY_STATE_GETTING };
    }

    AZStd::optional<GetEntitiesStatesServiceHandler::Response> GetEntitiesStatesServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        AZ::Outcome<SimulationInterfaces::MultipleEntitiesStates, SimulationInterfaces::FailedResult> outcome;

        GetEntitiesStatesServiceHandler::Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;

        const auto getFilterResult = SimulationInterfaces::Utils::GetEntityFiltersFromRequest<Request>(request);
        if (!getFilterResult.IsSuccess())
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
            response.result.error_message = getFilterResult.GetError().c_str();
            return response;
        }
        SimulationInterfaces::EntityFilters filter = getFilterResult.GetValue();
        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntitiesStates, filter);

        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = failedResult.m_errorCode;
            response.result.error_message = failedResult.m_errorString.c_str();
            return response;
        }

        const auto& multipleEntitiesStates = outcome.GetValue();

        response.entities.reserve(multipleEntitiesStates.size());
        response.states.reserve(multipleEntitiesStates.size());
        const auto simulatorFrameId = RegistryUtilities::GetSimulatorROS2Frame();
        for (auto& [entityName, entityState] : multipleEntitiesStates)
        {
            // entity name
            response.entities.push_back(entityName.c_str());
            // entity state
            simulation_interfaces::msg::EntityState simulationInterfacesEntityState;
            simulationInterfacesEntityState.header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
            simulationInterfacesEntityState.header.frame_id = simulatorFrameId.c_str();
            simulationInterfacesEntityState.pose = ROS2::ROS2Conversions::ToROS2Pose(entityState.m_pose);
            simulationInterfacesEntityState.twist.linear = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twistLinear);
            simulationInterfacesEntityState.twist.angular = ROS2::ROS2Conversions::ToROS2Vector3(entityState.m_twistAngular);
            response.states.push_back(simulationInterfacesEntityState);
        }

        return response;
    }
} // namespace ROS2SimulationInterfaces
