/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2WheelOdometry.h"
#include "Odometry/ROS2OdometryCovariance.h"
#include "VehicleModelComponent.h"
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace
    {
        const char* WheelOdometryMsgType = "nav_msgs::msg::Odometry";
    }

    void ROS2WheelOdometryComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2OdometryCovariance::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2WheelOdometryComponent, SensorBaseType>()
                ->Version(2)
                ->Field("Twist covariance", &ROS2WheelOdometryComponent::m_twistCovariance)
                ->Field("Pose covariance", &ROS2WheelOdometryComponent::m_poseCovariance);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2WheelOdometryComponent>("ROS2 Wheel Odometry Sensor", "Odometry sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2WheelOdometrySensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2WheelOdometrySensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2WheelOdometryComponent::m_twistCovariance,
                        "Twist covariance",
                        "Set ROS twist covariance")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2WheelOdometryComponent::m_poseCovariance,
                        "Pose covariance",
                        "Set ROS pose covariance");
            }
        }
    }

    ROS2WheelOdometryComponent::ROS2WheelOdometryComponent()
    {
        TopicConfiguration tc;
        const AZStd::string type = WheelOdometryMsgType;
        tc.m_type = type;
        tc.m_topic = "odom";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, tc));
    }

    void ROS2WheelOdometryComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("SkidSteeringModelService"));
    }

    void ROS2WheelOdometryComponent::OnOdometryEvent()
    {
        m_odometryMsg.pose.pose.position = ROS2Conversions::ToROS2Point(m_robotPose);
        m_odometryMsg.pose.pose.orientation = ROS2Conversions::ToROS2Quaternion(m_robotRotation);
        m_odometryMsg.pose.covariance = m_poseCovariance.GetRosCovariance();

        m_odometryPublisher->publish(m_odometryMsg);
    }

    void ROS2WheelOdometryComponent::OnPhysicsEvent(float physicsDeltaTime)
    {
        AZStd::pair<AZ::Vector3, AZ::Vector3> vt;

        VehicleDynamics::VehicleInputControlRequestBus::EventResult(
            vt, GetEntityId(), &VehicleDynamics::VehicleInputControlRequests::GetWheelsOdometry);

        m_odometryMsg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        m_odometryMsg.twist.twist.linear = ROS2Conversions::ToROS2Vector3(vt.first);
        m_odometryMsg.twist.twist.angular = ROS2Conversions::ToROS2Vector3(vt.second);
        m_odometryMsg.twist.covariance = m_twistCovariance.GetRosCovariance();

        if (m_sensorConfiguration.m_frequency > 0)
        {
            const auto updatePos = physicsDeltaTime * vt.first; // in meters
            const auto updateRot = physicsDeltaTime * vt.second; // in radians
            m_robotPose += m_robotRotation.TransformVector(updatePos);
            m_robotRotation *= AZ::Quaternion::CreateFromScaledAxisAngle(updateRot);
        }
    }

    void ROS2WheelOdometryComponent::Activate()
    {
        m_robotPose = AZ::Vector3{ 0 };
        m_robotRotation = AZ::Quaternion{ 0, 0, 0, 1 };

        // "odom" is globally fixed frame for all robots, no matter the namespace
        m_odometryMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
        m_odometryMsg.child_frame_id = GetFrameID().c_str();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

        const auto& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[WheelOdometryMsgType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_odometryPublisher = ros2Node->create_publisher<nav_msgs::msg::Odometry>(fullTopic.data(), publisherConfig.GetQoS());

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
                OnOdometryEvent();
            },
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime)
            {
                OnPhysicsEvent(physicsDeltaTime);
            });
    }

    void ROS2WheelOdometryComponent::Deactivate()
    {
        StopSensor();
        m_odometryPublisher.reset();
    }
} // namespace ROS2
