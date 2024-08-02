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

        const RaycastResult& lastScanResults = m_lidarCore.PerformRaycast();

        if (m_canRaycasterPublish || !m_sensorConfiguration.m_publishingEnabled)
        { // Skip publishing when it is disabled or can be handled by the raycaster.
            return;
        }

        PublishRaycastResults(lastScanResults);
    }

    void ROS2LidarSensorComponent::PublishRaycastResults(const RaycastResult& results)
    {
        const bool isIntensityEnabled = m_lidarCore.m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::Intensity;

        auto* ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        auto message = sensor_msgs::msg::PointCloud2();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.height = 1;
        message.width = results.m_points.size();

        sensor_msgs::PointCloud2Modifier modifier(message);
        if (isIntensityEnabled)
        {
            modifier.setPointCloud2Fields(
                4,
                "x",
                1,
                sensor_msgs::msg::PointField::FLOAT32,
                "y",
                1,
                sensor_msgs::msg::PointField::FLOAT32,
                "z",
                1,
                sensor_msgs::msg::PointField::FLOAT32,
                "intensity",
                1,
                sensor_msgs::msg::PointField::FLOAT32);
        }
        else
        {
            modifier.setPointCloud2Fields(
                3,
                "x",
                1,
                sensor_msgs::msg::PointField::FLOAT32,
                "y",
                1,
                sensor_msgs::msg::PointField::FLOAT32,
                "z",
                1,
                sensor_msgs::msg::PointField::FLOAT32);
        }
        modifier.resize(results.m_points.size());

        sensor_msgs::PointCloud2Iterator<float> xIt(message, "x");
        sensor_msgs::PointCloud2Iterator<float> yIt(message, "y");
        sensor_msgs::PointCloud2Iterator<float> zIt(message, "z");

        AZStd::optional<sensor_msgs::PointCloud2Iterator<float>> intensityIt;
        if (isIntensityEnabled)
        {
            intensityIt = sensor_msgs::PointCloud2Iterator<float>(message, "intensity");
        }

        const auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        const auto inverseLidarTM = entityTransform->GetWorldTM().GetInverse();
        for (size_t i = 0; i < results.m_points.size(); ++i)
        {
            const AZ::Vector3 globalPoint = inverseLidarTM.TransformPoint(results.m_points[i]);
            *xIt = globalPoint.GetX();
            *yIt = globalPoint.GetY();
            *zIt = globalPoint.GetZ();

            if (isIntensityEnabled)
            {
                *intensityIt.value() = results.m_intensities[i];
                ++intensityIt.value();
            }

            ++xIt; ++yIt; ++ zIt;
        }

        m_pointCloudPublisher->publish(message);
    }
} // namespace ROS2
