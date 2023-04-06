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
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2Lidar2DSensorComponent, ROS2SensorComponent>()
                ->Version(1)
                ->Field("lidarModel", &ROS2Lidar2DSensorComponent::m_lidarModel)
                ->Field("lidarImplementation", &ROS2Lidar2DSensorComponent::m_lidarSystem)
                ->Field("LidarParameters", &ROS2Lidar2DSensorComponent::m_lidarParameters)
                ->Field("IgnoreLayer", &ROS2Lidar2DSensorComponent::m_ignoreLayer)
                ->Field("IgnoredLayerIndex", &ROS2Lidar2DSensorComponent::m_ignoredLayerIndex)
                ->Field("ExcludedEntities", &ROS2Lidar2DSensorComponent::m_excludedEntities)
                ->Field("PointsAtMax", &ROS2Lidar2DSensorComponent::m_addPointsAtMax);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2Lidar2DSensorComponent>("ROS2 Lidar 2D Sensor", "Lidar 2D sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &ROS2Lidar2DSensorComponent::m_lidarModel, "Lidar Model", "Lidar model")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ROS2Lidar2DSensorComponent::OnLidarModelSelected)
                    ->EnumAttribute(LidarTemplate::LidarModel::Custom2DLidar, "Custom Lidar 2D")
                    ->EnumAttribute(LidarTemplate::LidarModel::Slamtec_RPLIDAR_S1, "Slamtec RPLIDAR S1")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2Lidar2DSensorComponent::m_lidarSystem,
                        "Lidar Implementation",
                        "Select a lidar implementation out of registered ones.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ROS2Lidar2DSensorComponent::OnLidarImplementationSelected)
                    ->Attribute(AZ::Edit::Attributes::StringList, &ROS2Lidar2DSensorComponent::FetchLidarSystemList)
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ROS2Lidar2DSensorComponent::m_lidarParameters,
                        "Lidar parameters",
                        "Configuration of Custom lidar")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2Lidar2DSensorComponent::IsConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2Lidar2DSensorComponent::m_ignoreLayer,
                        "Ignore layer",
                        "Should we ignore selected layer index")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2Lidar2DSensorComponent::IsIgnoredLayerConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2Lidar2DSensorComponent::m_ignoredLayerIndex,
                        "Ignored layer index",
                        "Layer index to ignore")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2Lidar2DSensorComponent::IsIgnoredLayerConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2Lidar2DSensorComponent::m_excludedEntities,
                        "Excluded Entities",
                        "List of entities excluded from raycasting.")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2Lidar2DSensorComponent::IsEntityExclusionVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2Lidar2DSensorComponent::m_addPointsAtMax,
                        "Points at Max",
                        "If set true LiDAR will produce points at max range for free space")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2Lidar2DSensorComponent::IsMaxPointsConfigurationVisible);
            }
        }
    }
    static AZStd::string GetDefaultLidarSystem()
    {
        const auto& lidarSystemList = LidarRegistrarInterface::Get()->GetRegisteredLidarSystems();
        if (lidarSystemList.empty())
        {
            AZ_Warning("ROS2LidarSensorComponent", false, "No LIDAR system for the sensor to use.");
            return {};
        }
        return lidarSystemList.front();
    }

    void ROS2Lidar2DSensorComponent::FetchLidarImplementationFeatures()
    {
        if (m_lidarSystem.empty())
        {
            m_lidarSystem = GetDefaultLidarSystem();
        }
        const auto* lidarMetaData = LidarRegistrarInterface::Get()->GetLidarSystemMetaData(m_lidarSystem);
        AZ_Warning("ROS2Lidar2DSensorComponent", lidarMetaData, "No metadata for \"%s\"", m_lidarSystem.c_str());
        if (lidarMetaData)
        {
            m_lidarSystemFeatures = LidarRegistrarInterface::Get()->GetLidarSystemMetaData(m_lidarSystem)->m_features;
        }
    }

    bool ROS2Lidar2DSensorComponent::IsConfigurationVisible() const
    {
        return m_lidarModel == LidarTemplate::LidarModel::Custom2DLidar;
    }

    bool ROS2Lidar2DSensorComponent::IsIgnoredLayerConfigurationVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::CollisionLayers;
    }

    bool ROS2Lidar2DSensorComponent::IsEntityExclusionVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::EntityExclusion;
    }

    bool ROS2Lidar2DSensorComponent::IsMaxPointsConfigurationVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::MaxRangePoints;
    }

    AZStd::vector<AZStd::string> ROS2Lidar2DSensorComponent::FetchLidarSystemList()
    {
        FetchLidarImplementationFeatures();
        return LidarRegistrarInterface::Get()->GetRegisteredLidarSystems();
    }

    AZ::Crc32 ROS2Lidar2DSensorComponent::OnLidarModelSelected()
    {
        m_lidarParameters = LidarTemplateUtils::GetTemplate(m_lidarModel);
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::Crc32 ROS2Lidar2DSensorComponent::OnLidarImplementationSelected()
    {
        FetchLidarImplementationFeatures();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    void ROS2Lidar2DSensorComponent::ConnectToLidarRaycaster()
    {
        if (auto raycasterId = m_implementationToRaycasterMap.find(m_lidarSystem); raycasterId != m_implementationToRaycasterMap.end())
        {
            m_lidarRaycasterId = raycasterId->second;
            return;
        }

        m_lidarRaycasterId = LidarId::CreateNull();
        LidarSystemRequestBus::EventResult(
            m_lidarRaycasterId, AZ_CRC(m_lidarSystem), &LidarSystemRequestBus::Events::CreateLidar, GetEntityId());
        AZ_Assert(!m_lidarRaycasterId.IsNull(), "Could not access selected Lidar System.");

        m_implementationToRaycasterMap.emplace(m_lidarSystem, m_lidarRaycasterId);
    }

    void ROS2Lidar2DSensorComponent::ConfigureLidarRaycaster()
    {
        FetchLidarImplementationFeatures();
        LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRayOrientations, m_lastRotations);
        LidarRaycasterRequestBus::Event(
            m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRayRange, m_lidarParameters.m_maxRange);
        LidarRaycasterRequestBus::Event(
            m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureMinimumRayRange, m_lidarParameters.m_minRange);

        if (m_lidarSystemFeatures & LidarSystemFeatures::CollisionLayers)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureLayerIgnoring, m_ignoreLayer, m_ignoredLayerIndex);
        }

        if (m_lidarSystemFeatures & LidarSystemFeatures::EntityExclusion)
        {
            LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ExcludeEntities, m_excludedEntities);
        }

        if (m_lidarSystemFeatures & LidarSystemFeatures::MaxRangePoints)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureMaxRangePointAddition, m_addPointsAtMax);
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

    void ROS2Lidar2DSensorComponent::Visualise()
    {
        if (m_visualisationPoints.empty())
        {
            return;
        }

        if (m_drawQueue)
        {
            const uint8_t pixelSize = 2;
            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
            drawArgs.m_verts = m_visualisationPoints.data();
            drawArgs.m_vertCount = m_visualisationPoints.size();
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

        if (m_sensorConfiguration.m_visualise)
        {
            auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        }

        m_lastRotations = LidarTemplateUtils::PopulateRayRotations(m_lidarParameters);

        FetchLidarImplementationFeatures();
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
        if (m_sensorConfiguration.m_visualise) {
            requestedFlags |= RaycastResultFlags::Points;
        }
        LidarRaycasterRequestBus::EventResult(
            m_lastScanResults,
            m_lidarRaycasterId,
            &LidarRaycasterRequestBus::Events::PerformRaycastWithFlags,
            entityTransform->GetWorldTM(), requestedFlags);
        if (m_lastScanResults.m_ranges.empty())
        {
            AZ_TracePrintf("Lidar Sensor Component", "No results from raycast\n");
            return;
        }

        if (m_sensorConfiguration.m_visualise)
        { // Store points for visualisation purposes, in global frame
            m_visualisationPoints = m_lastScanResults.m_points;
        }

        // TODO(mzak): possibly unneccessary post-proc?
        const auto inverseLidarTM = entityTransform->GetWorldTM().GetInverse();
        for (auto& point : m_lastScanResults.m_points)
        {
            point = inverseLidarTM.TransformPoint(point);
        }

        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::LaserScan();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.angle_min = AZ::DegToRad(m_lidarParameters.m_minHAngle);
        message.angle_max = AZ::DegToRad(m_lidarParameters.m_maxHAngle);
        message.angle_increment = (message.angle_max - message.angle_min) / aznumeric_cast<float>(m_lidarParameters.m_numberOfIncrements);

        message.range_min = m_lidarParameters.m_minRange;
        message.range_max = m_lidarParameters.m_maxRange;
        message.scan_time = 1.f / m_sensorConfiguration.m_frequency;
        message.time_increment = 0.0f;

        message.ranges.assign(m_lastScanResults.m_ranges.begin(), m_lastScanResults.m_ranges.end());
        m_laserScanPublisher->publish(message);
    }
} // namespace ROS2
