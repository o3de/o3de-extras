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
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kLaserScanType = "sensor_msgs::msg::LaserScan";
    }

    void ROS2Lidar2DSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        Lidar2DSensorConfiguration::Reflect(context);

        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2Lidar2DSensorComponent, ROS2SensorComponent>()->Version(2)->Field(
                "lidarConfiguration", &ROS2Lidar2DSensorComponent::m_lidarConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2Lidar2DSensorComponent>("ROS2 Lidar 2D Sensor", "Lidar 2D sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2Lidar2DSensorComponent::m_lidarConfiguration,
                        "Lidar configuration",
                        "Lidar configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void ROS2Lidar2DSensorComponent::ConnectToLidarRaycaster()
    {
        if (auto raycasterId = m_implementationToRaycasterMap.find(m_lidarConfiguration.m_lidarSystem);
            raycasterId != m_implementationToRaycasterMap.end())
        {
            m_lidarRaycasterId = raycasterId->second;
            return;
        }

        m_lidarRaycasterId = LidarId::CreateNull();
        LidarSystemRequestBus::EventResult(
            m_lidarRaycasterId, AZ_CRC(m_lidarConfiguration.m_lidarSystem), &LidarSystemRequestBus::Events::CreateLidar, GetEntityId());
        AZ_Assert(!m_lidarRaycasterId.IsNull(), "Could not access selected Lidar System.");

        m_implementationToRaycasterMap.emplace(m_lidarConfiguration.m_lidarSystem, m_lidarRaycasterId);
    }

    void ROS2Lidar2DSensorComponent::ConfigureLidarRaycaster()
    {
        m_lidarConfiguration.FetchLidarImplementationFeatures();
        LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRayOrientations, m_lastRotations);
        LidarRaycasterRequestBus::Event(
            m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRayRange, m_lidarConfiguration.m_lidarParameters.m_maxRange);
        LidarRaycasterRequestBus::Event(
            m_lidarRaycasterId,
            &LidarRaycasterRequestBus::Events::ConfigureMinimumRayRange,
            m_lidarConfiguration.m_lidarParameters.m_minRange);

        if (m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::Noise &&
            m_lidarConfiguration.m_lidarParameters.m_isNoiseEnabled)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigureNoiseParameters,
                m_lidarConfiguration.m_lidarParameters.m_noiseParameters.m_angularNoiseStdDev,
                m_lidarConfiguration.m_lidarParameters.m_noiseParameters.m_distanceNoiseStdDevBase,
                m_lidarConfiguration.m_lidarParameters.m_noiseParameters.m_distanceNoiseStdDevRisePerMeter);
        }

        RaycastResultFlags requestedFlags = RaycastResultFlags::Ranges;
        if (m_sensorConfiguration.m_visualize)
        {
            requestedFlags |= RaycastResultFlags::Points;
        }
        LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRaycastResultFlags, requestedFlags);

        if (m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::CollisionLayers)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigureLayerIgnoring,
                m_lidarConfiguration.m_ignoreLayer,
                m_lidarConfiguration.m_ignoredLayerIndex);
        }

        if (m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::EntityExclusion)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ExcludeEntities, m_lidarConfiguration.m_excludedEntities);
        }

        if (m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::MaxRangePoints)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigureMaxRangePointAddition,
                m_lidarConfiguration.m_addPointsAtMax);
        }
    }

    ROS2Lidar2DSensorComponent::ROS2Lidar2DSensorComponent()
    {
        TopicConfiguration ls;
        AZStd::string type = Internal::kLaserScanType;
        ls.m_type = type;
        ls.m_topic = "ls";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, ls));
    }

    ROS2Lidar2DSensorComponent::ROS2Lidar2DSensorComponent(
        const SensorConfiguration& sensorConfiguration, const Lidar2DSensorConfiguration& lidarConfiguration)
        : m_lidarConfiguration(lidarConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2Lidar2DSensorComponent::Visualize()
    {
        if (m_visualizationPoints.empty())
        {
            return;
        }

        if (m_drawQueue)
        {
            const uint8_t pixelSize = 2;
            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
            drawArgs.m_verts = m_visualizationPoints.data();
            drawArgs.m_vertCount = m_visualizationPoints.size();
            drawArgs.m_colors = &AZ::Colors::Red;
            drawArgs.m_colorCount = 1;
            drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
            drawArgs.m_size = pixelSize;
            m_drawQueue->DrawPoints(drawArgs);
        }
    }

    void ROS2Lidar2DSensorComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

        const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kLaserScanType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_laserScanPublisher = ros2Node->create_publisher<sensor_msgs::msg::LaserScan>(fullTopic.data(), publisherConfig.GetQoS());

        if (m_sensorConfiguration.m_visualize)
        {
            auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        }

        m_lastRotations = LidarTemplateUtils::PopulateRayRotations(m_lidarConfiguration.m_lidarParameters);

        m_lidarConfiguration.FetchLidarImplementationFeatures();
        ConnectToLidarRaycaster();
        ConfigureLidarRaycaster();

        ROS2SensorComponent::Activate();
    }

    void ROS2Lidar2DSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_laserScanPublisher.reset();
    }

    void ROS2Lidar2DSensorComponent::FrequencyTick()
    {
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();

        RaycastResultFlags requestedFlags = RaycastResultFlags::Ranges;
        if (m_sensorConfiguration.m_visualize)
        {
            requestedFlags |= RaycastResultFlags::Points;
        }
        LidarRaycasterRequestBus::EventResult(
            m_lastScanResults, m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::PerformRaycast, entityTransform->GetWorldTM());
        if (m_lastScanResults.m_ranges.empty())
        {
            AZ_TracePrintf("Lidar Sensor Component", "No results from raycast\n");
            return;
        }

        if (m_sensorConfiguration.m_visualize)
        { // Store points for visualization purposes, in global frame
            m_visualizationPoints = m_lastScanResults.m_points;
        }

        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::LaserScan();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.angle_min = AZ::DegToRad(m_lidarConfiguration.m_lidarParameters.m_minHAngle);
        message.angle_max = AZ::DegToRad(m_lidarConfiguration.m_lidarParameters.m_maxHAngle);
        message.angle_increment =
            (message.angle_max - message.angle_min) / aznumeric_cast<float>(m_lidarConfiguration.m_lidarParameters.m_numberOfIncrements);

        message.range_min = m_lidarConfiguration.m_lidarParameters.m_minRange;
        message.range_max = m_lidarConfiguration.m_lidarParameters.m_maxRange;
        message.scan_time = 1.f / m_sensorConfiguration.m_frequency;
        message.time_increment = 0.0f;

        message.ranges.assign(m_lastScanResults.m_ranges.begin(), m_lastScanResults.m_ranges.end());
        m_laserScanPublisher->publish(message);
    }
} // namespace ROS2
