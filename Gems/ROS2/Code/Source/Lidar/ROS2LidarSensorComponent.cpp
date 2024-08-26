/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <Lidar/PointCloudMessageBuilder.h>
#include <Lidar/ROS2LidarSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ROS2
{
    namespace
    {
        const char* PointCloudType = "sensor_msgs::msg::PointCloud2";
    }

    void ROS2LidarSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2LidarSensorComponent, SensorBaseType>()->Version(3)->Field(
                "lidarCore", &ROS2LidarSensorComponent::m_lidarCore);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ROS2LidarSensorComponent>("ROS2 Lidar Sensor", "Lidar sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2LidarSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2LidarSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2LidarSensorComponent::m_lidarCore,
                        "Lidar configuration",
                        "Lidar configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void ROS2LidarSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    ROS2LidarSensorComponent::ROS2LidarSensorComponent()
        : m_lidarCore(LidarTemplateUtils::Get3DModels())
    {
        TopicConfiguration pc;
        AZStd::string type = PointCloudType;
        pc.m_type = type;
        pc.m_topic = "pc";
        m_sensorConfiguration.m_frequency = 10.f;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
    }

    ROS2LidarSensorComponent::ROS2LidarSensorComponent(
        const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration)
        : m_lidarCore(lidarConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2LidarSensorComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();
        m_lidarCore.Init(GetEntityId());

        m_lidarRaycasterId = m_lidarCore.GetLidarRaycasterId();
        m_canRaycasterPublish = false;
        if (m_lidarCore.m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::PointcloudPublishing)
        {
            LidarRaycasterRequestBus::EventResult(
                m_canRaycasterPublish, m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::CanHandlePublishing);
        }

        if (m_canRaycasterPublish)
        {
            const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[PointCloudType];
            auto* ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();

            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigurePointCloudPublisher,
                ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic),
                ros2Frame->GetFrameID().data(),
                publisherConfig.GetQoS());
        }
        else
        {
            auto ros2Node = ROS2Interface::Get()->GetNode();
            AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

            const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[PointCloudType];
            AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
            m_pointCloudPublisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic.data(), publisherConfig.GetQoS());
        }

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                FrequencyTick();
            },
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_visualize)
                {
                    return;
                }
                m_lidarCore.VisualizeResults();
            });
    }

    void ROS2LidarSensorComponent::Deactivate()
    {
        StopSensor();
        m_pointCloudPublisher.reset();
        m_lidarCore.Deinit();
        ROS2SensorComponentBase::Deactivate();
    }

    void ROS2LidarSensorComponent::FrequencyTick()
    {
        if (m_canRaycasterPublish && m_sensorConfiguration.m_publishingEnabled)
        {
            const builtin_interfaces::msg::Time timestamp = ROS2Interface::Get()->GetROSTimestamp();
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::UpdatePublisherTimestamp,
                aznumeric_cast<AZ::u64>(timestamp.sec) * aznumeric_cast<AZ::u64>(1.0e9f) + timestamp.nanosec);
        }

        AZStd::optional<RaycastResults> lastScanResults = m_lidarCore.PerformRaycast();

        if (!lastScanResults.has_value() || m_canRaycasterPublish || !m_sensorConfiguration.m_publishingEnabled)
        {
            return;
        }

        PublishRaycastResults(lastScanResults.value());
    }

    void ROS2LidarSensorComponent::PublishRaycastResults(const RaycastResults& results)
    {
        const bool isIntensityEnabled = m_lidarCore.m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::Intensity;

        auto builder = PointCloud2MessageBuilder(
            GetEntity()->FindComponent<ROS2FrameComponent>()->GetFrameID(), ROS2Interface::Get()->GetROSTimestamp(), results.GetCount());

        builder.AddField("x", sensor_msgs::msg::PointField::FLOAT32)
            .AddField("y", sensor_msgs::msg::PointField::FLOAT32)
            .AddField("z", sensor_msgs::msg::PointField::FLOAT32);

        if (isIntensityEnabled)
        {
            builder.AddField("intensity", sensor_msgs::msg::PointField::FLOAT32);
        }
        sensor_msgs::msg::PointCloud2 message = builder.Get();


        sensor_msgs::PointCloud2Iterator<float> messageXIt(message, "x");
        sensor_msgs::PointCloud2Iterator<float> messageYIt(message, "y");
        sensor_msgs::PointCloud2Iterator<float> messageZIt(message, "z");

        const auto positionField = results.GetConstFieldSpan<RaycastResultFlags::Point>().value();
        auto positionIt = positionField.begin();

        AZStd::optional<sensor_msgs::PointCloud2Iterator<float>> messageIntensityIt;
        AZStd::optional<RaycastResults::FieldSpan<RaycastResultFlags::Intensity>::const_iterator> intensityIt;
        if (isIntensityEnabled)
        {
            messageIntensityIt = sensor_msgs::PointCloud2Iterator<float>(message, "intensity");
            intensityIt = results.GetConstFieldSpan<RaycastResultFlags::Intensity>().value().begin();
        }

        const auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        const auto inverseLidarTM = entityTransform->GetWorldTM().GetInverse();
        for (size_t i = 0; i < results.GetCount(); ++i)
        {
            AZ::Vector3 point = *positionIt;
            const AZ::Vector3 globalPoint = inverseLidarTM.TransformPoint(point);
            *messageXIt = globalPoint.GetX();
            *messageYIt = globalPoint.GetY();
            *messageZIt = globalPoint.GetZ();

            if (messageIntensityIt.has_value() && intensityIt.has_value())
            {
                *messageIntensityIt.value() = *intensityIt.value();
                ++intensityIt.value();
                ++messageIntensityIt.value();
            }

            ++positionIt;
            ++messageXIt;
            ++messageYIt;
            ++messageZIt;
        }

        m_pointCloudPublisher->publish(message);
    }
} // namespace ROS2
