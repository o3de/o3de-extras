/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SetEntityStateServiceHandler.h"
#include "SpawnServiceUtils.h"
#include <ROS2/TF/TransformInterface.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/RegistryUtils.h>
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
        const AZStd::string entityName = request.entity.c_str();
        const auto simulatorFrameId = RegistryUtilities::GetSimulatorROS2Frame();
        const AZStd::string_view messageFrameId{ request.state.header.frame_id.c_str(), request.state.header.frame_id.length() };

        // Resolve frame transform offset once (needed for both pose and twist)
        AZ::Transform transformOffset = AZ::Transform::CreateIdentity();
#if SIMULATION_INTERFACES_MAJOR_API_VERSION >= 2
        if ((request.set_pose || request.set_twist) && !messageFrameId.empty() && simulatorFrameId != messageFrameId)
#else
        if (!messageFrameId.empty() && simulatorFrameId != messageFrameId)
#endif
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

        SimulationInterfaces::EntityState entityState;
#if SIMULATION_INTERFACES_MAJOR_API_VERSION >= 2
        // Fetch current state so unflagged fields are preserved
        {
            AZ::Outcome<SimulationInterfaces::EntityState, SimulationInterfaces::FailedResult> currentState;
            SimulationInterfaces::SimulationEntityManagerRequestBus::BroadcastResult(
                currentState, &SimulationInterfaces::SimulationEntityManagerRequests::GetEntityState, entityName);
            if (currentState.IsSuccess())
            {
                entityState = currentState.GetValue();
            }
        }

        if (request.set_pose)
        {
            const AZ::Transform requestedPose = ROS2::ROS2Conversions::FromROS2Pose(request.state.pose);
            if (const auto poseValidation = SpawnServiceUtils::ValidateTransformNormalized(requestedPose); !poseValidation.IsSuccess())
            {
                Response response;
                response.result.result = simulation_interfaces::srv::SetEntityState::Response::INVALID_POSE;
                response.result.error_message = poseValidation.GetError().c_str();
                return response;
            }
            entityState.m_pose = transformOffset *
                AZ::Transform::CreateFromQuaternionAndTranslation(requestedPose.GetRotation().GetNormalized(), requestedPose.GetTranslation());
        }

        if (request.set_twist)
        {
            entityState.m_twistAngular = transformOffset.TransformVector(ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.angular));
            entityState.m_twistLinear = transformOffset.TransformVector(ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.linear));
        }
#else
        const AZ::Transform requestedPose = ROS2::ROS2Conversions::FromROS2Pose(request.state.pose);
        if (const auto poseValidation = SpawnServiceUtils::ValidateTransformNormalized(requestedPose); !poseValidation.IsSuccess())
        {
            Response response;
            response.result.result = simulation_interfaces::srv::SetEntityState::Response::INVALID_POSE;
            response.result.error_message = poseValidation.GetError().c_str();
            return response;
        }
        entityState.m_pose = transformOffset *
            AZ::Transform::CreateFromQuaternionAndTranslation(requestedPose.GetRotation().GetNormalized(), requestedPose.GetTranslation());
        entityState.m_twistAngular = transformOffset.TransformVector(ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.angular));
        entityState.m_twistLinear = transformOffset.TransformVector(ROS2::ROS2Conversions::FromROS2Vector3(request.state.twist.linear));
#endif

        AZ::Outcome<void, SimulationInterfaces::FailedResult> outcome;
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
