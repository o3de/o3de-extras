/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GetNamedPosesServiceHandler.h"
#include "SimulationInterfaces/ROS2SimulationInterfacesRequestBus.h"
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/NamedPoseManagerRequestBus.h>
#include <SimulationInterfaces/Result.h>
#include <SimulationInterfaces/TagFilter.h>
#include <simulation_interfaces/msg/named_pose.hpp>

namespace ROS2SimulationInterfaces
{

    AZStd::unordered_set<SimulationFeatureType> GetNamedPosesServiceHandler::GetProvidedFeatures()
    {
        return AZStd::unordered_set<SimulationFeatureType>{ SimulationFeatures::NAMED_POSES, SimulationFeatures::POSE_BOUNDS };
    }

    AZStd::optional<GetNamedPosesServiceHandler::Response> GetNamedPosesServiceHandler::HandleServiceRequest(
        const std::shared_ptr<rmw_request_id_t> header, const Request& request)
    {
        using namespace SimulationInterfaces;
        AZ::Outcome<NamedPoseList, FailedResult> namedPosesO3DE;
        TagFilter tagFilter;
        tagFilter.m_mode = request.tags.filter_mode;
        for (const auto& tag : request.tags.tags)
        {
            tagFilter.m_tags.emplace(tag.c_str());
        }

        NamedPoseManagerRequestBus::BroadcastResult(namedPosesO3DE, &NamedPoseManagerRequests::GetNamedPoses, tagFilter);
        Response response;
        if (!namedPosesO3DE.IsSuccess())
        {
            response.result.result = namedPosesO3DE.GetError().m_errorCode;
            response.result.error_message = namedPosesO3DE.GetError().m_errorString.c_str();
        }
        else
        {
            response.result.result = simulation_interfaces::msg::Result::RESULT_OK;
            for (const auto& namedPose : namedPosesO3DE.GetValue())
            {
                simulation_interfaces::msg::NamedPose ros2NamedPose;
                ros2NamedPose.pose = ROS2::ROS2Conversions::ToROS2Pose(namedPose.m_pose);
                ros2NamedPose.description = namedPose.m_description.c_str();
                ros2NamedPose.name = namedPose.m_name.c_str();
                for (const auto& tag : namedPose.m_tags)
                {
                    ros2NamedPose.tags.emplace_back(tag.c_str());
                }

                response.poses.push_back(ros2NamedPose);
            }
        }
        return response;
    }
} // namespace ROS2SimulationInterfaces
