/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "VelocitySplinePublisher.h"
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>

namespace ROS2
{

    void VelocitySplinePublisher::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VelocitySplinePublisher>()
                ->Version(1)

                ->Field("LinearSpeedFactor", &VelocitySplinePublisher::m_linearSpeedFactor)
                ->Field("AngularSpeedFactor", &VelocitySplinePublisher::m_angularSpeedFactor)
                ->Field("CrossTrackFactor", &VelocitySplinePublisher::m_crossTrackFactor)
                ->Field("LookAheadDistance", &VelocitySplinePublisher::m_lookAheadDistance)
                ->Field("cmdTopic", &VelocitySplinePublisher::m_cmdTopicConfiguration)
                ->Field("RobotBaselink", &VelocitySplinePublisher::m_baselinkEntityId)
                ->Field("DrawInGame", &VelocitySplinePublisher::m_drawInGame);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VelocitySplinePublisher>("VelocitySplinePublisher", "VelocitySplinePublisher")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VelocitySplinePublisher::m_linearSpeedFactor,
                        "LinearSpeedFactor",
                        "LinearSpeedFactor")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VelocitySplinePublisher::m_angularSpeedFactor,
                        "AngularSpeedFactor",
                        "AngularSpeedFactor")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VelocitySplinePublisher::m_cmdTopicConfiguration,
                        "Velocity Topic publisher",
                        "Velocity Topic publisher")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &VelocitySplinePublisher::m_crossTrackFactor, "CrossTrackFactor", "CrossTrackFactor")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VelocitySplinePublisher::m_lookAheadDistance,
                        "LookAheadDistance",
                        "LookAheadDistance")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId, &VelocitySplinePublisher::m_baselinkEntityId, "RobotBaselink", "RobotBaselink")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VelocitySplinePublisher::m_drawInGame,
                        "Draw in Game",
                        "Draw track and goal in game.");
            }
        }
    }

    void VelocitySplinePublisher::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void VelocitySplinePublisher::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        const auto& topicName = m_cmdTopicConfiguration.m_topic;
        const auto& qos = m_cmdTopicConfiguration.GetQoS();
        m_cmdPublisher = ros2Node->create_publisher<geometry_msgs::msg::Twist>(topicName.data(), qos);
        AZ::EntityBus::Handler::BusConnect(m_baselinkEntityId);
        if (m_drawInGame)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(m_entity->GetId());
        }
    }

    void VelocitySplinePublisher::Deactivate()
    {
        if (m_drawInGame)
        {
            AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect(m_entity->GetId());
        }
        AZ::EntityBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    float VelocitySplinePublisher::GetAngle(const AZ::Vector3& v1, const AZ::Vector3& v2)
    {
        return atan2(v1.Cross(v2).Dot(AZ::Vector3::CreateAxisZ(1.0f)), v1.Dot(v2));
    }

    void VelocitySplinePublisher::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        AZ::Transform splineTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);

        // get robot location in spline's frame
        AZ::Transform robotLocationWorld{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(robotLocationWorld, m_baselinkEntityId, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Transform robotLocationSpline = splineTransform.GetInverse() * robotLocationWorld;

        // query spline for nearest address
        const AZ::PositionSplineQueryResult splineQuery =
            splinePtr->GetNearestAddressPosition(robotLocationSpline.TransformPoint(AZ::Vector3::CreateAxisX(m_lookAheadDistance)));
        const AZ::Vector3 position = splinePtr->GetPosition(splineQuery.m_splineAddress);
        const AZ::Vector3 tangent = splinePtr->GetTangent(splineQuery.m_splineAddress);
        const AZ::Vector3 normal = splinePtr->GetNormal(splineQuery.m_splineAddress);

        // construct ideal pose as SE(3) - in spline space
        const AZ::Matrix3x3 rot = AZ::Matrix3x3::CreateFromColumns(tangent, normal, tangent.Cross(normal));
        const AZ::Transform goalTransform = AZ::Transform::CreateFromMatrix3x3AndTranslation(rot, position);
        m_idealGoal = splineTransform * goalTransform;

        // calculate robot location in goal space
        AZ::Vector3 robotLocationInGoalSpace = m_idealGoal.GetInverse().TransformPoint(robotLocationWorld.GetTranslation());

        // calculate linear velocity
        float linearVelocity = 0;
        if (splineQuery.m_splineAddress.m_segmentIndex != splinePtr->GetSegmentCount())
        {
            linearVelocity = m_linearSpeedFactor;
        }

        // cross track error
        const float crossTrackError = robotLocationInGoalSpace.GetY();

        // calculate bearing error
        const AZ::Vector3 robotDirectionSpline = robotLocationSpline.GetBasisX();
        const float bearingError = GetAngle(robotDirectionSpline, tangent);

        AZ_Printf(
            "ROS2",
            " %d %f %f %f",
            splineQuery.m_splineAddress.m_segmentIndex,
            robotLocationInGoalSpace.GetX(),
            robotLocationInGoalSpace.GetY(),
            robotLocationInGoalSpace.GetZ());

        geometry_msgs::msg::Twist cmd;
        cmd.angular.z = bearingError * m_angularSpeedFactor - crossTrackError * m_crossTrackFactor;
        cmd.linear.x = linearVelocity;
        m_cmdPublisher->publish(cmd);
    }

    void VelocitySplinePublisher::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        AZ::Transform splineTransform{ AZ::Transform::CreateIdentity() };
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);

        constexpr float step = 0.01f;
        for (float f = step; f < 1.0f; f += step)
        {
            auto pos1 = splinePtr->GetAddressByFraction(f);
            auto pos2 = splinePtr->GetAddressByFraction(f - step);

            debugDisplay.SetColor(AZ::Colors::Red);
            debugDisplay.DrawLine(
                splineTransform.TransformPoint(splinePtr->GetPosition(pos1)), splineTransform.TransformPoint(splinePtr->GetPosition(pos2)));
        }
        debugDisplay.SetColor(AZ::Colors::White);
        debugDisplay.DrawPoint(m_idealGoal.GetTranslation(), 4);
        debugDisplay.SetColor(AZ::Colors::Red);
        debugDisplay.DrawLine(m_idealGoal.GetTranslation(), m_idealGoal.TransformPoint(AZ::Vector3::CreateAxisX(1.0f)));
        debugDisplay.SetColor(AZ::Colors::Green);
        debugDisplay.DrawLine(m_idealGoal.GetTranslation(), m_idealGoal.TransformPoint(AZ::Vector3::CreateAxisY(1.0f)));
        debugDisplay.SetColor(AZ::Colors::Blue);
        debugDisplay.DrawLine(m_idealGoal.GetTranslation(), m_idealGoal.TransformPoint(AZ::Vector3::CreateAxisZ(1.0f)));
    }

    void VelocitySplinePublisher::OnEntityActivated(const AZ::EntityId& entityId)
    {
        if (entityId == m_baselinkEntityId)
        {
            AZ::TickBus::Handler::BusConnect();
        }
    }

} // namespace ROS2