/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <Lidar/ROS2Lidar2DSensorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace
    {
        const char* LaserScanType = "sensor_msgs::msg::LaserScan";
    }

    void ROS2Lidar2DSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2Lidar2DSensorComponent, SensorBaseType>()->Version(3)->Field(
                "lidarCore", &ROS2Lidar2DSensorComponent::m_lidarCore);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ROS2Lidar2DSensorComponent>("ROS2 Lidar 2D Sensor", "Lidar 2D sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2Lidar2DSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2Lidar2DSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2Lidar2DSensorComponent::m_lidarCore,
                        "Lidar configuration",
                        "Lidar configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2Lidar2DSensorComponent::ROS2Lidar2DSensorComponent()
        : m_lidarCore(LidarTemplateUtils::Get2DModels())
    {
        TopicConfiguration ls;
        AZStd::string type = LaserScanType;
        ls.m_type = type;
        ls.m_topic = "scan";
        m_sensorConfiguration.m_frequency = 10.f;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, ls));
    }
    ROS2Lidar2DSensorComponent::ROS2Lidar2DSensorComponent(
        const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration)
        : m_lidarCore(lidarConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2Lidar2DSensorComponent::Activate()
    {
        m_lidarCore.Init(GetEntityId());

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

        const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[LaserScanType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_laserScanPublisher = ros2Node->create_publisher<sensor_msgs::msg::LaserScan>(fullTopic.data(), publisherConfig.GetQoS());

        StartSensor(
            m_sensorConfiguration.m_frequency,
            [this]([[maybe_unused]] auto&&... args)
            {
                if (!m_sensorConfiguration.m_publishingEnabled)
                {
                    return;
                }
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

    void ROS2Lidar2DSensorComponent::Deactivate()
    {
        StopSensor();
        m_laserScanPublisher.reset();
        m_lidarCore.Deinit();
    }

    void ROS2Lidar2DSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2Lidar2DSensorComponent::FrequencyTick()
    {
        RaycastResult lastScanResults = m_lidarCore.PerformRaycast();

        auto* ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        auto message = sensor_msgs::msg::LaserScan();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.angle_min = AZ::DegToRad(m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minHAngle);
        message.angle_max = AZ::DegToRad(m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxHAngle);
        message.angle_increment = (message.angle_max - message.angle_min) /
            aznumeric_cast<float>(m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_numberOfIncrements);

        message.range_min = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minRange;
        message.range_max = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxRange;
        message.scan_time = 1.f / m_sensorConfiguration.m_frequency;
        message.time_increment = 0.0f;

        message.ranges.assign(lastScanResults.m_ranges.begin(), lastScanResults.m_ranges.end());
        m_laserScanPublisher->publish(message);
    }
} // namespace ROS2
