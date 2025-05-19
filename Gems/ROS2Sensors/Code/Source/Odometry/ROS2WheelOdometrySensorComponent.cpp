/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2WheelOdometrySensorComponent.h"
#include <AzCore/Serialization/Json/JsonSerialization.h>
#include <AzCore/Serialization/Json/JsonSerializationResult.h>
#include <AzCore/Serialization/Json/RegistrationContext.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2Sensors/Odometry/ROS2OdometryCovariance.h>

namespace ROS2Sensors
{
    namespace
    {
        const char* WheelOdometryMsgType = "nav_msgs::msg::Odometry";
    }

    // Manual conversion between version without a configuration struct and with a configuration struct.
    // This is done to maintain backwards compatibility with older versions of the component.
    // This function checks if in the prefab there exits a member called "Twist covariance" or "Pose covariance".
    // If it does, it will load the values into the new configuration struct.
    // If it doesn't, it will treat the loaded json as it would load the new version of the component.
    // Checking old members is used instead of checking if there is a member called "Odometry configuration"
    // because is default values are used O3DE does not create an empty member and initializes the component with default values.
    // Meaning if there does not exist a member called "Odometry configuration" this component could use old values or default ones.
    AZ::JsonSerializationResult::Result JsonROS2WheelOdometryComponentConfigSerializer::Load(
        void* outputValue, const AZ::Uuid& outputValueTypeId, const rapidjson::Value& inputValue, AZ::JsonDeserializerContext& context)
    {
        namespace JSR = AZ::JsonSerializationResult;

        auto configInstance = reinterpret_cast<ROS2WheelOdometryComponent*>(outputValue);
        AZ_Assert(configInstance, "Output value for JsonROS2WheelOdometryComponentConfigSerializer can't be null.");

        JSR::ResultCode result(JSR::Tasks::ReadField);

        const bool hasOldMembers = inputValue.HasMember("Twist covariance") || inputValue.HasMember("Pose covariance");
        if (hasOldMembers)
        {
            {
                JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                    &configInstance->m_odometryConfiguration.m_poseCovariance,
                    azrtti_typeid<decltype(configInstance->m_odometryConfiguration.m_poseCovariance)>(),
                    inputValue,
                    "Pose covariance",
                    context);

                result.Combine(componentIdLoadResult);
            }

            {
                JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                    &configInstance->m_odometryConfiguration.m_twistCovariance,
                    azrtti_typeid<decltype(configInstance->m_odometryConfiguration.m_twistCovariance)>(),
                    inputValue,
                    "Twist covariance",
                    context);

                result.Combine(componentIdLoadResult);
            }
        }
        else
        {
            {
                JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                    &configInstance->m_odometryConfiguration,
                    azrtti_typeid<decltype(configInstance->m_odometryConfiguration)>(),
                    inputValue,
                    "Odometry configuration",
                    context);

                result.Combine(componentIdLoadResult);
            }
        }
        {
            JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                &configInstance->m_sensorConfiguration,
                azrtti_typeid<decltype(configInstance->m_sensorConfiguration)>(),
                inputValue,
                "Sensor configuration",
                context);

            result.Combine(componentIdLoadResult);
        }

        return context.Report(
            result,
            result.GetProcessing() != JSR::Processing::Halted ? "Successfully loaded ROS2FrameComponent information."
                                                              : "Failed to load ROS2FrameComponent information.");
    }

    AZ_CLASS_ALLOCATOR_IMPL(JsonROS2WheelOdometryComponentConfigSerializer, AZ::SystemAllocator);

    void ROS2WheelOdometryComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto jsonContext = azrtti_cast<AZ::JsonRegistrationContext*>(context))
        {
            jsonContext->Serializer<JsonROS2WheelOdometryComponentConfigSerializer>()->HandlesType<ROS2WheelOdometryComponent>();
        }

        WheelOdometrySensorConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2WheelOdometryComponent, SensorBaseType>()->Version(3)->Field(
                "Odometry configuration", &ROS2WheelOdometryComponent::m_odometryConfiguration);

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
                        &ROS2WheelOdometryComponent::m_odometryConfiguration,
                        "Odometry configuration",
                        "Odometry sensor configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2WheelOdometryComponent::ROS2WheelOdometryComponent()
    {
        ROS2::TopicConfiguration tc;
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
        m_odometryMsg.pose.pose.position = ROS2::ROS2Conversions::ToROS2Point(m_robotPose);
        m_odometryMsg.pose.pose.orientation = ROS2::ROS2Conversions::ToROS2Quaternion(m_robotRotation);
        m_odometryMsg.pose.covariance = m_odometryConfiguration.m_poseCovariance.GetRosCovariance();

        m_odometryPublisher->publish(m_odometryMsg);
    }

    void ROS2WheelOdometryComponent::OnPhysicsEvent(float physicsDeltaTime)
    {
        AZStd::pair<AZ::Vector3, AZ::Vector3> vt;

        // Temporarily disabled until the ROS2Controllers gem is available
        // VehicleDynamics::VehicleInputControlRequestBus::EventResult(
        //     vt, GetEntityId(), &VehicleDynamics::VehicleInputControlRequests::GetWheelsOdometry);

        m_odometryMsg.header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_odometryMsg.twist.twist.linear = ROS2::ROS2Conversions::ToROS2Vector3(vt.first);
        m_odometryMsg.twist.twist.angular = ROS2::ROS2Conversions::ToROS2Vector3(vt.second);
        m_odometryMsg.twist.covariance = m_odometryConfiguration.m_twistCovariance.GetRosCovariance();

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
        ROS2SensorComponentBase::Activate();
        m_robotPose = AZ::Vector3{ 0 };
        m_robotRotation = AZ::Quaternion{ 0, 0, 0, 1 };

        // "odom" is globally fixed frame for all robots, no matter the namespace
        m_odometryMsg.header.frame_id = ROS2::ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
        m_odometryMsg.child_frame_id = GetFrameID().c_str();

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

        const auto& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[WheelOdometryMsgType];
        const auto fullTopic = ROS2::ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
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

        WheelOdometryConfigurationRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2WheelOdometryComponent::Deactivate()
    {
        WheelOdometryConfigurationRequestBus::Handler::BusDisconnect(GetEntityId());
        StopSensor();
        m_odometryPublisher.reset();
        ROS2SensorComponentBase::Deactivate();
    }

    const WheelOdometrySensorConfiguration ROS2WheelOdometryComponent::GetConfiguration() const
    {
        return m_odometryConfiguration;
    }

    void ROS2WheelOdometryComponent::SetConfiguration(const WheelOdometrySensorConfiguration& configuration)
    {
        m_odometryConfiguration = configuration;
    }

    ROS2OdometryCovariance ROS2WheelOdometryComponent::GetPoseCovariance() const
    {
        return m_odometryConfiguration.m_poseCovariance;
    }

    void ROS2WheelOdometryComponent::SetPoseCovariance(const ROS2OdometryCovariance& covariance)
    {
        m_odometryConfiguration.m_poseCovariance = covariance;
    }

    ROS2OdometryCovariance ROS2WheelOdometryComponent::GetTwistCovariance() const
    {
        return m_odometryConfiguration.m_twistCovariance;
    }

    void ROS2WheelOdometryComponent::SetTwistCovariance(const ROS2OdometryCovariance& covariance)
    {
        m_odometryConfiguration.m_twistCovariance = covariance;
    }

} // namespace ROS2Sensors
