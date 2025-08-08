/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SetEntityStateServiceHandler.h"
#include <ROS2/TF/TransformInterface.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> SetEntityStateServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::ENTITY_STATE_SETTING };
    }

    AZStd::optional<SetEntityStateServiceHandler::Response> SetEntityStateServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        const auto simulatorFrameId = RegistryUtilities::GetSimulatorROS2Frame();
        const AZStd::string_view messageFrameId{ request.state.header.frame_id.c_str(), request.state.header.frame_id.length() };
        AZ::Transform transformOffset = AZ::Transform::CreateIdentity();
        if (!messageFrameId.empty() && simulatorFrameId != messageFrameId)
        {
            const builtin_interfaces::msg::Time time = request.state.header.stamp;
            auto transformInterface = ROS2::TFInterface::Get();
            AZ_Assert(transformInterface, "TFInterface is not available, cannot set entity state without transform offset.");
            const auto transformOutcome = transformInterface->GetTransform(simulatorFrameId, messageFrameId, time);

            if (transformOutcome.IsSuccess())
            {
                transformOffset = transformOutcome.GetValue();
            }
            else
            {
                Response response;
                response.result.result = simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED;
                response.result.error_message = transformOutcome.GetError().c_str();
                return response;
            }
        }
        AZ::Outcome<void, SimulationInterfaces::FailedResult> outcome;
        AZStd::string entityName = request.entity.c_str();

        SimulationInterfaces::EntityState entityState;

        entityState.m_pose = transformOffset * ROS2::ROS2Conversions::FromROS2Pose(request.state.pose);
        entityState.m_twistAngular = transformOffset.TransformVector(ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.angular));
        entityState.m_twistLinear = transformOffset.TransformVector(ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.linear));

        SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
            outcome, &SimulationInterfaces::SimulationEntityManagerRequests::SetEntityState, entityName, entityState);

        Response response;
        response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
        if (!outcome.IsSuccess())
        {
            const auto& failedResult = outcome.GetError();
            response.result.result = failedResult.m_errorCode;
            response.result.error_message = failedResult.m_errorString.c_str();
        }

        return response;
    }
} // namespace ROS2SimulationInterfaces
