/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FingerGripperComponent.h"
#include "Utils.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/JointsManipulationRequests.h>
#include <Utilities/JointUtilities.h>
#include <imgui/imgui.h>

namespace ROS2
{
    void FingerGripperComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("GripperService"));
    }
    void FingerGripperComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("GripperService"));
    }

    void FingerGripperComponent::Activate()
    {
        m_grippingInProgress = false;
        m_initialised = false;
        m_cancelled = false;
        m_ImGuiPosition = 0.0f;
        m_stallingFor = 0.0f;
        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        GripperRequestBus::Handler::BusConnect(GetEntityId());
    }

    void FingerGripperComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        GripperRequestBus::Handler::BusDisconnect(GetEntityId());
    }

    void FingerGripperComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<FingerGripperComponent, AZ::Component>()
                ->Field("VelocityEpsilon", &FingerGripperComponent::m_velocityEpsilon)
                ->Field("DistanceEpsilon", &FingerGripperComponent::m_goalTolerance)
                ->Field("StallTime", &FingerGripperComponent::m_stallTime)
                ->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<FingerGripperComponent>("FingerGripperComponent", "Component controlling a finger gripper.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "FingerGripperComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/FingerGripperComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/FingerGripperComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FingerGripperComponent::m_velocityEpsilon,
                        "Velocity Epsilon",
                        "The maximum velocity to consider the gripper as stalled.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FingerGripperComponent::m_goalTolerance,
                        "Goal tolerance",
                        "Goal is considered reached if the gripper is at this distance or closer")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FingerGripperComponent::m_stallTime,
                        "Stall Time",
                        "The time to wait before considering the gripper as stalled.");
            }
        }
    }

    ManipulationJoints& FingerGripperComponent::GetFingerJoints()
    {
        m_fingerJoints.clear();
        m_rootOfArticulation = Utils::GetRootOfArticulation(GetEntityId());
        AZ_Warning(
            "FingerGripperComponent",
            m_rootOfArticulation.IsValid(),
            "Entity %s is not part of an articulation.",
            GetEntity()->GetName().c_str());
        ManipulationJoints allJoints;
        if (m_rootOfArticulation.IsValid())
        {
            JointsManipulationRequestBus::EventResult(allJoints, m_rootOfArticulation, &JointsManipulationRequests::GetJoints);
        }
        AZStd::vector<AZ::EntityId> descendantIds;
        AZ::TransformBus::EventResult(descendantIds, GetEntityId(), &AZ::TransformBus::Events::GetAllDescendants);

        for (AZ::EntityId descendant : descendantIds)
        {
            AZStd::string jointName = ROS2::Utils::GetJointName(descendant);
            if (!jointName.empty())
            {
                AZ_Printf("FingerGripperComponent", "Adding finger joint %s", jointName.c_str());
                m_fingerJoints[jointName] = allJoints[jointName];
            }
        }

        return m_fingerJoints;
    }

    void FingerGripperComponent::SetPosition(float position, float maxEffort)
    {
        if (m_fingerJoints.empty())
        {
            return;
        }

        float targetPosition = position;
        for (auto& [jointName, jointInfo] : m_fingerJoints)
        {
            AZ::Outcome<void, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, m_rootOfArticulation, &JointsManipulationRequests::MoveJointToPosition, jointName, targetPosition);
            if (!result.IsSuccess())
            {
                AZ_Warning(
                    "FingerGripperComponent",
                    result,
                    "Joint move cannot be realized: %s for %s ",
                    result.GetError().c_str(),
                    jointName.c_str());
            }
        }

        float oneMaxEffort = maxEffort / m_fingerJoints.size();
        for (auto& [jointName, jointInfo] : m_fingerJoints)
        {
            AZ::Outcome<void, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, m_rootOfArticulation, &JointsManipulationRequests::SetMaxJointEffort, jointName, oneMaxEffort);
            if (!result.IsSuccess())
            {
                AZ_Warning(
                    "FingerGripperComponent",
                    result,
                    "Setting a max force for a joint cannot be realized: %s for %s ",
                    result.GetError().c_str(),
                    jointName.c_str());
            }
        }
    }

    AZ::Outcome<void, AZStd::string> FingerGripperComponent::GripperCommand(float position, float maxEffort)
    {
        if (maxEffort == 0.0f)
        { // The moveit panda demo fills the max effort fields with 0s, but we want to exert effort
            maxEffort = AZStd::numeric_limits<float>::infinity();
        }

        m_grippingInProgress = true;
        m_desiredPosition = position;
        m_stallingFor = 0.0f;
        m_cancelled = false;

        SetPosition(position, maxEffort);

        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> FingerGripperComponent::CancelGripperCommand()
    {
        m_grippingInProgress = false;
        m_cancelled = true;
        SetPosition(0.0f, AZStd::numeric_limits<float>::infinity());
        return AZ::Success();
    }

    bool FingerGripperComponent::HasGripperCommandBeenCancelled() const
    {
        return m_cancelled;
    }

    float FingerGripperComponent::GetGripperPosition() const
    {
        float gripperPosition = 0.0f;
        if (m_fingerJoints.empty())
        {
            return gripperPosition;
        }

        for (const auto& [jointName, _] : m_fingerJoints)
        {
            AZ::Outcome<JointPosition, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(
                result, m_rootOfArticulation, &JointsManipulationRequests::GetJointPosition, jointName);
            gripperPosition += result.GetValueOr(0.f);
        }

        return gripperPosition / m_fingerJoints.size();
    }

    float FingerGripperComponent::GetGripperEffort() const
    {
        float gripperEffort = 0.0f;
        for (const auto& [jointName, _] : m_fingerJoints)
        {
            AZ::Outcome<JointEffort, AZStd::string> result;
            JointsManipulationRequestBus::EventResult(result, m_rootOfArticulation, &JointsManipulationRequests::GetJointEffort, jointName);
            if (result)
            {
                gripperEffort += result.GetValue();
            }
        }
        return gripperEffort;
    }

    bool FingerGripperComponent::IsGripperVelocity0() const
    {
        for (const auto& [jointName, _] : m_fingerJoints)
        {
            AZ::Outcome<JointEffort, AZStd::string> velocityResult;
            JointsManipulationRequestBus::EventResult(
                velocityResult, m_rootOfArticulation, &JointsManipulationRequests::GetJointVelocity, jointName);
            if (velocityResult && AZStd::abs(velocityResult.GetValue()) > m_velocityEpsilon)
            {
                return false;
            }
        }

        return true;
    }

    bool FingerGripperComponent::IsGripperNotMoving() const
    {
        return m_stallingFor > m_stallTime;
    }

    bool FingerGripperComponent::HasGripperReachedGoal() const
    {
        return !m_grippingInProgress || AZStd::abs(GetGripperPosition() - m_desiredPosition) < m_goalTolerance;
    }

    void FingerGripperComponent::OnImGuiUpdate()
    {
        ImGui::Begin("FingerGripperDebugger");

        ImGui::SliderFloat("Target Position", &m_ImGuiPosition, 0.0f, 0.1f);

        if (ImGui::Button("Execute Command"))
        {
            GripperCommand(m_ImGuiPosition, AZStd::numeric_limits<float>::infinity());
        }

        ImGui::End();
    }

    void FingerGripperComponent::OnTick([[maybe_unused]] float delta, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        if (!m_initialised)
        {
            m_initialised = true;
            GetFingerJoints();
            SetPosition(0.0f, AZStd::numeric_limits<float>::infinity());
        }

        if (IsGripperVelocity0())
        {
            m_stallingFor += delta;
        }
        else
        {
            m_stallingFor = 0.0f;
        }
    }
} // namespace ROS2
