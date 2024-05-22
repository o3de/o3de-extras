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
            if (m_lidarCore.m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::Segmentation) {
                 m_segmentationClassesPublisher = ros2Node->create_publisher<vision_msgs::msg::LabelInfo>(
                    ROS2Names::GetNamespacedName(GetNamespace(), "segmentation_classes").data(),
                    publisherConfig.GetQoS());
            }
        }

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

    void ROS2LidarSensorComponent::Deactivate()
    {
        StopSensor();
        m_pointCloudPublisher.reset();
        m_lidarCore.Deinit();
    }

    void ROS2LidarSensorComponent::FrequencyTick()
    {
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();

        if (m_canRaycasterPublish)
        {
            const builtin_interfaces::msg::Time timestamp = ROS2Interface::Get()->GetROSTimestamp();
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::UpdatePublisherTimestamp,
                aznumeric_cast<AZ::u64>(timestamp.sec) * aznumeric_cast<AZ::u64>(1.0e9f) + timestamp.nanosec);
        }

        auto lastScanResults = m_lidarCore.PerformRaycast();

        if (m_canRaycasterPublish)
        { // Skip publishing when it can be handled by the raycaster.
            return;
        }

        const auto inverseLidarTM = entityTransform->GetWorldTM().GetInverse();
        for (auto& point : lastScanResults.m_points)
        {
            point = inverseLidarTM.TransformPoint(point);
        }
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::PointCloud2();
        const auto pointCount = lastScanResults.m_points.size();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.height = 1;
        message.width = pointCount;
        message.point_step = 3 * sizeof(float);

        AZStd::array<const char*, 3> pointFieldNames = { "x", "y", "z" };

        for (int i = 0; i < pointFieldNames.size(); i++)
        {
            sensor_msgs::msg::PointField pf;
            pf.name = pointFieldNames[i];
            pf.offset = i * 4;
            pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
            pf.count = 1;
            message.fields.push_back(pf);
        }

        if (lastScanResults.m_ids.has_value())
        {
            sensor_msgs::msg::PointField pfId;
            pfId.name = "entity_id";
            pfId.offset = message.point_step;
            pfId.datatype = sensor_msgs::msg::PointField::INT32;
            pfId.count = 1;
            message.fields.push_back(pfId);
            constexpr auto fieldLength = sizeof(int32_t);
            message.point_step += fieldLength;
        }
        AZStd::array<AZ::Color, LidarSensorConfiguration::maxClass> colorLookupTable;
        if (lastScanResults.m_classes.has_value())
        {
            colorLookupTable = m_lidarCore.m_lidarConfiguration.GenerateSegmentationColorsLookupTable();

            AZStd::array<const char*, 3> colorFieldNames = { "r", "g", "b" };

            for (int i = 0; i < colorFieldNames.size(); i++)
            {
                sensor_msgs::msg::PointField pf;
                pf.name = colorFieldNames[i];
                pf.offset = message.point_step;
                pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
                pf.count = 1;
                message.fields.push_back(pf);
                message.point_step += sizeof(float);
            }

            sensor_msgs::msg::PointField pfClassId;
            pfClassId.name = "class_id";
            pfClassId.offset = message.point_step;
            pfClassId.datatype = sensor_msgs::msg::PointField::UINT8;
            pfClassId.count = 1;
            message.fields.push_back(pfClassId);
            message.point_step += sizeof(uint8_t);
        }

        const auto sizeInBytes = pointCount * message.point_step;
        message.data.resize(sizeInBytes);

        float x, y, z;
        for (int i = 0; i < pointCount; ++i)
        {
            // to avoid alignment issues, we copy the data field by field
            x = lastScanResults.m_points[i].GetX();
            y = lastScanResults.m_points[i].GetY();
            z = lastScanResults.m_points[i].GetZ();

            memcpy(&message.data[i * message.point_step], &x, sizeof(float));
            memcpy(&message.data[i * message.point_step + sizeof(float)], &y, sizeof(float));
            memcpy(&message.data[i * message.point_step + 2 * sizeof(float)], &z, sizeof(float));
            uint32_t nextFieldOffset = 3 * sizeof(float);

            if (lastScanResults.m_ids.has_value())
            {
                memcpy(&message.data[i * message.point_step + nextFieldOffset], &lastScanResults.m_ids.value()[i], sizeof(int32_t));
                nextFieldOffset += sizeof(int32_t);
            }

            if (lastScanResults.m_classes.has_value())
            {
                const AZ::Color color = colorLookupTable[lastScanResults.m_classes.value()[i]];
                for (int j = 0; j < 3; j++)
                {
                    float chanelValue = color.GetElement(j);
                    memcpy(&message.data[i * message.point_step + nextFieldOffset], &chanelValue, sizeof(float));
                    nextFieldOffset += sizeof(float);
                }
                memcpy(&message.data[i * message.point_step + nextFieldOffset], &lastScanResults.m_classes.value()[i], sizeof(uint8_t));
                nextFieldOffset += sizeof(uint8_t);
            }
        }
        message.row_step = message.width * message.point_step;
        AZ_Assert(message.row_step * message.height == sizeInBytes, "Inconsistency in the size of point cloud data");

        m_pointCloudPublisher->publish(message);

        if (m_segmentationClassesPublisher)
        {
            vision_msgs::msg::LabelInfo segmentationClasses;
            for (const auto&segmentation_class : m_lidarCore.m_lidarConfiguration.m_segmentationClasses)
            {
                vision_msgs::msg::VisionClass visionClass;
                visionClass.class_id = segmentation_class.m_classId;
                visionClass.class_name = segmentation_class.m_className.c_str();
                segmentationClasses.class_map.push_back(visionClass);
            }
            m_segmentationClassesPublisher->publish(segmentationClasses);
        }
    }
} // namespace ROS2
