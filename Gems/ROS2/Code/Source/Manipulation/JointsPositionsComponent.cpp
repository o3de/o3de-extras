/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsPositionsComponent.h"
#include "JointPositionsSubscriptionHandler.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <ROS2/ROS2GemUtilities.h>
#include <Utilities/ArticulationsUtilities.h>

namespace ROS2
{
    JointsPositionsComponent::JointsPositionsComponent(
        const TopicConfiguration& topicConfiguration, const AZStd::vector<AZStd::string>& jointNames)
        : m_topicConfiguration(topicConfiguration)
        , m_jointNames(jointNames)
    {
    }

    void JointsPositionsComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsPositionsComponent::Activate()
    {
        m_jointPositionsSubscriptionHandler = AZStd::make_unique<JointPositionsSubscriptionHandler>(
            [this](const JointPositionsSubscriptionHandler::MessageType& message)
            {
                ProcessPositionControlMessage(message);
            });
        m_jointPositionsSubscriptionHandler->Activate(GetEntity(), m_topicConfiguration);

        AZ::TickBus::Handler::BusConnect();
    }

    void JointsPositionsComponent::Deactivate()
    {
        if (m_jointPositionsSubscriptionHandler)
        {
            m_jointPositionsSubscriptionHandler->Deactivate();
            m_jointPositionsSubscriptionHandler.reset();
        }

        AZ::TickBus::Handler::BusDisconnect();
    }

    void JointsPositionsComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsPositionsComponent, AZ::Component>()
                ->Version(0)
                ->Field("topicConfiguration", &JointsPositionsComponent::m_topicConfiguration)
                ->Field("jointNames", &JointsPositionsComponent::m_jointNames);
        }
    }

    void JointsPositionsComponent::OnTick([[maybe_unused]] float delta, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        if (!m_rootOfArticulation.IsValid())
        {
            m_rootOfArticulation = Utils::GetRootOfArticulation(GetEntityId());
            AZ_Warning(
                "JointsPositionsComponent",
                m_rootOfArticulation.IsValid(),
                "Entity %s is not part of an articulation.",
                GetEntity()->GetName().c_str());

            AZ::TickBus::Handler::BusDisconnect();
        }
    }

    void JointsPositionsComponent::ProcessPositionControlMessage(const std_msgs::msg::Float64MultiArray& message)
    {
        if (message.data.size() != m_jointNames.size())
        {
            AZ_Error(
                "JointsPositionsComponent",
                false,
                "PositionController: command size %d does not match the number of joints %d",
                message.data.size(),
                m_jointNames.size());
            return;
        }

        auto commandIter = message.data.cbegin();
        for (const auto& jointName : m_jointNames)
        {
            AZ::Outcome<void, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, m_rootOfArticulation, &JointsManipulationRequests::MoveJointToPosition, jointName, *commandIter);
            if (!result.IsSuccess())
            {
                AZ_Error(
                    "JointsPositionsComponent",
                    result,
                    "PositionController: command failed for joint %s: ",
                    jointName.c_str(),
                    result.GetError().c_str());
            }
        }
    }

} // namespace ROS2
