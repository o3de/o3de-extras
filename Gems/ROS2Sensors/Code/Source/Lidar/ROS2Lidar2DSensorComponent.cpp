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
#include <ROS2/Clock/ROS2ClockRequestBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2NamesBus.h>

namespace ROS2Sensors
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
        ROS2::TopicConfiguration ls;
        AZStd::string type = LaserScanType;
        ls.m_type = type;
        ls.m_topic = "scan";
        m_sensorConfiguration.m_frequency = 10.f;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, ls));
    }
    ROS2Lidar2DSensorComponent::ROS2Lidar2DSensorComponent(
        const ROS2::SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration)
        : m_lidarCore(lidarConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2Lidar2DSensorComponent::Activate()
    {
        ROS2SensorComponentBase::Activate();
        m_lidarCore.Init(GetEntityId());

        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

        const ROS2::TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[LaserScanType];
        AZStd::string fullTopic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            fullTopic, &ROS2::ROS2NamesRequestBus::Events::GetNamespacedName, GetNamespace(), publisherConfig.m_topic);
        m_laserScanPublisher = ros2Node->create_publisher<sensor_msgs::msg::LaserScan>(fullTopic.data(), publisherConfig.GetQoS());

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

        LidarConfigurationRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2Lidar2DSensorComponent::Deactivate()
    {
        LidarConfigurationRequestBus::Handler::BusDisconnect(GetEntityId());
        StopSensor();
        m_laserScanPublisher.reset();
        m_lidarCore.Deinit();
        ROS2SensorComponentBase::Deactivate();
    }

    void ROS2Lidar2DSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2Lidar2DSensorComponent::FrequencyTick()
    {
        AZStd::optional<RaycastResults> results = m_lidarCore.PerformRaycast();

        if (!results.has_value() || !m_sensorConfiguration.m_publishingEnabled)
        {
            return;
        }

        PublishRaycastResults(results.value());
    }

    void ROS2Lidar2DSensorComponent::PublishRaycastResults(const RaycastResults& results)
    {
        const bool isIntensityEnabled = m_lidarCore.m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::Intensity;

        auto* ros2Frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>();
        auto message = sensor_msgs::msg::LaserScan();
        message.header.frame_id = ros2Frame->GetNamespacedFrameID().data();
        message.header.stamp = ROS2::ROS2ClockInterface::Get()->GetROSTimestamp();
        message.angle_min = AZ::DegToRad(m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minHAngle);
        message.angle_max = AZ::DegToRad(m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxHAngle);
        message.angle_increment = (message.angle_max - message.angle_min) /
            aznumeric_cast<float>(m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_numberOfIncrements);

        message.range_min = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minRange;
        message.range_max = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxRange;
        message.scan_time = 1.f / m_sensorConfiguration.m_frequency;
        message.time_increment = 0.0f;

        const auto rangeField = results.GetConstFieldSpan<RaycastResultFlags::Range>().value();
        message.ranges.assign(rangeField.begin(), rangeField.end());
        if (isIntensityEnabled)
        {
            const auto intensityField = results.GetConstFieldSpan<RaycastResultFlags::Intensity>().value();
            message.intensities.assign(intensityField.begin(), intensityField.end());
        }

        m_laserScanPublisher->publish(message);
    }

    void ROS2Lidar2DSensorComponent::SetConfiguration(const LidarSensorConfiguration& configuration)
    {
        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration = configuration;
        m_lidarCore.Init(GetEntityId());
    }

    const LidarSensorConfiguration ROS2Lidar2DSensorComponent::GetConfiguration()
    {
        return m_lidarCore.m_lidarConfiguration;
    }

    AZStd::string ROS2Lidar2DSensorComponent::GetModelName()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_name;
    }

    void ROS2Lidar2DSensorComponent::SetModelName(const AZStd::string& name)
    {
        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_lidarModelName = name;
        m_lidarCore.m_lidarConfiguration.FetchLidarModelConfiguration();
        // Unknown lidar models are set to custom lidar model with a given name
        m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_name = name;
        m_lidarCore.Init(GetEntityId());
    }

    bool ROS2Lidar2DSensorComponent::IsSegmentationEnabled()
    {
        return m_lidarCore.m_lidarConfiguration.m_isSegmentationEnabled;
    }

    void ROS2Lidar2DSensorComponent::SetSegmentationEnabled(bool enabled)
    {
        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_isSegmentationEnabled = enabled;
        m_lidarCore.Init(GetEntityId());
    }

    bool ROS2Lidar2DSensorComponent::IsAddPointsAtMaxEnabled()
    {
        return m_lidarCore.m_lidarConfiguration.m_addPointsAtMax;
    }

    void ROS2Lidar2DSensorComponent::SetAddPointsAtMaxEnabled(bool addPoints)
    {
        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_addPointsAtMax = addPoints;
        m_lidarCore.Init(GetEntityId());
    }

    bool ROS2Lidar2DSensorComponent::Is2D()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_is2D;
    }

    float ROS2Lidar2DSensorComponent::GetMinHAngle()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minHAngle;
    }

    void ROS2Lidar2DSensorComponent::SetMinHAngle(float angle)
    {
        if (angle < -180.0f || angle > 180.0f)
        {
            AZ_Error("ROS2Lidar2DSensorComponent", false, "Min horizontal angle must be between -180 and 180 degrees.");
            return;
        }

        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minHAngle = angle;
        m_lidarCore.Init(GetEntityId());
    }

    float ROS2Lidar2DSensorComponent::GetMaxHAngle()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxHAngle;
    }

    void ROS2Lidar2DSensorComponent::SetMaxHAngle(float angle)
    {
        if (angle < -180.0f || angle > 180.0f)
        {
            AZ_Error("ROS2Lidar2DSensorComponent", false, "Max horizontal angle must be between -180 and 180 degrees.");
            return;
        }

        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxHAngle = angle;
        m_lidarCore.Init(GetEntityId());
    }

    unsigned int ROS2Lidar2DSensorComponent::GetNumberOfIncrements()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_numberOfIncrements;
    }

    void ROS2Lidar2DSensorComponent::SetNumberOfIncrements(unsigned int increments)
    {
        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_numberOfIncrements = increments;
        m_lidarCore.Init(GetEntityId());
    }

    float ROS2Lidar2DSensorComponent::GetMinRange()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minRange;
    }

    void ROS2Lidar2DSensorComponent::SetMinRange(float range)
    {
        const float maxRange = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxRange;
        if (range < 0.0f || range > maxRange)
        {
            AZ_Error("ROS2Lidar2DSensorComponent", false, "Min range cannot be less than 0 or greater than max range (%f).", maxRange);
            return;
        }

        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minRange = range;
        m_lidarCore.Init(GetEntityId());
    }

    float ROS2Lidar2DSensorComponent::GetMaxRange()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxRange;
    }

    void ROS2Lidar2DSensorComponent::SetMaxRange(float range)
    {
        const float minRange = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_minRange;
        if (range < minRange)
        {
            AZ_Error("ROS2Lidar2DSensorComponent", false, "Max range cannot be less than min range (%f).", minRange);
            return;
        }

        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_maxRange = range;
        m_lidarCore.Init(GetEntityId());
    }

    const LidarTemplate::NoiseParameters& ROS2Lidar2DSensorComponent::GetNoiseParameters()
    {
        return m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_noiseParameters;
    }

    void ROS2Lidar2DSensorComponent::SetNoiseParameters(const LidarTemplate::NoiseParameters& params)
    {
        if (params.m_angularNoiseStdDev < 0.0f || params.m_angularNoiseStdDev > 180.0f || params.m_distanceNoiseStdDevBase < 0.0f ||
            params.m_distanceNoiseStdDevRisePerMeter < 0.0f)
        {
            AZ_Error("ROS2Lidar2DSensorComponent", false, "Invalid noise parameters provided.");
            return;
        }

        m_lidarCore.Deinit();
        m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_noiseParameters = params;
        m_lidarCore.Init(GetEntityId());
    }
} // namespace ROS2Sensors
