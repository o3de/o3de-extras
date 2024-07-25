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
#include <AzCore/Jobs/Algorithms.h>

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
            if (m_lidarCore.m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::Segmentation && m_lidarCore.m_lidarConfiguration.m_isSegmentationEnabled) {
                 m_segmentationClassesPublisher = ros2Node->create_publisher<vision_msgs::msg::LabelInfo>(
                    ROS2Names::GetNamespacedName(GetNamespace(), "segmentation_classes").data(),
                    publisherConfig.GetQoS());
            }
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
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();

        if (m_canRaycasterPublish && m_sensorConfiguration.m_publishingEnabled)
        {
            const builtin_interfaces::msg::Time timestamp = ROS2Interface::Get()->GetROSTimestamp();
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::UpdatePublisherTimestamp,
                aznumeric_cast<AZ::u64>(timestamp.sec) * aznumeric_cast<AZ::u64>(1.0e9f) + timestamp.nanosec);
        }

        auto lastScanResults = m_lidarCore.PerformRaycast();

        if (m_canRaycasterPublish || !m_sensorConfiguration.m_publishingEnabled)
        { // Skip publishing when it is disabled or can be handled by the raycaster.
            return;
        }

        const auto inverseLidarTM = entityTransform->GetWorldTM().GetInverse();
        for (auto& point : lastScanResults.m_points)
        {
            point = inverseLidarTM.TransformPoint(point);
        }

        AZStd::vector<AZStd::function<unsigned char*(unsigned char* pointBeginPointer, const size_t pointIdx)>> pointFillers;
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::PointCloud2();
        const auto pointCount = lastScanResults.m_points.size();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.height = 1;
        message.width = pointCount;
        message.point_step = 0;

        auto AddField = [&message](const char* name, const size_t fieldSize, const sensor_msgs::msg::PointField::_datatype_type datatype, const size_t count) {
            sensor_msgs::msg::PointField pf;
            pf.name = name;
            pf.offset = message.point_step;
            pf.datatype = datatype;
            pf.count = count;
            message.fields.push_back(pf);
            message.point_step += count * fieldSize;
        };

        auto HandleXYZ = [&AddField, &lastScanResults]() -> AZStd::function<unsigned char*(
            unsigned char* pointBeginPointer,
            const size_t pointIdx)>
        {
            for (auto&& pointFieldName : { "x", "y", "z" })
            {
                AddField(pointFieldName, sizeof(float), sensor_msgs::msg::PointField::FLOAT32, 1);
            }

            return [&lastScanResults](unsigned char* pointBeginPointer, const size_t pointIdx) -> unsigned char* {
                memcpy(pointBeginPointer, &lastScanResults.m_points[pointIdx], 3 * sizeof(float));
                pointBeginPointer += 3 * sizeof(float);
                return pointBeginPointer;
            };
        };

        auto HandleEntityId = [&AddField, &lastScanResults,this]() -> AZStd::optional<AZStd::function<unsigned char*(
            unsigned char* pointBeginPointer,
            const size_t pointIdx)>>
        {
            if (!(this->m_lidarCore.m_lidarConfiguration.m_isSegmentationEnabled && lastScanResults.m_ids.has_value() && lastScanResults.
                m_ids.value().size() == lastScanResults.m_points.size()))
            {
                return {};
            }

            constexpr size_t fieldSize = sizeof(int32_t);
            AddField("entity_id", fieldSize, sensor_msgs::msg::PointField::INT32, 1);

            return { [&lastScanResults](unsigned char* pointBeginPointer, size_t pointIdx) -> unsigned char* {
                memcpy(pointBeginPointer, &lastScanResults.m_ids.value()[pointIdx], fieldSize);
                pointBeginPointer += fieldSize;
                return pointBeginPointer;
            } };
        };

        auto HandleClassAndClassColor = [&AddField, &lastScanResults,this](
            ) -> AZStd::optional<AZStd::function<unsigned char*(unsigned char* pointBeginPointer, const size_t pointIdx)>>
        {
            if (!(this->m_lidarCore.m_lidarConfiguration.m_isSegmentationEnabled && lastScanResults.m_classes.has_value()))
            {
                return {};
            }

            constexpr size_t classIdSize = sizeof(uint8_t);
            AddField("class_id", classIdSize, sensor_msgs::msg::PointField::UINT8, 1);
            AddField("rgba", sizeof(uint32_t), sensor_msgs::msg::PointField::UINT32, 1);

            AZStd::array<AZ::Color, LidarSensorConfiguration::maxClass> colorLookupTable = m_lidarCore.m_lidarConfiguration.
                GenerateSegmentationColorsLookupTable();

            return { [&lastScanResults,colorLookupTable](unsigned char* pointBeginPointer, const size_t pointIdx) -> unsigned char* {
                memcpy(pointBeginPointer, &lastScanResults.m_classes.value()[pointIdx], classIdSize);
                pointBeginPointer += classIdSize;
                const AZ::Color color = colorLookupTable[lastScanResults.m_classes.value()[pointIdx]];
                unsigned int rgbColor = 0;
                for (int j = 0; j < 4; j++)
                {
                    float channelValue = color.GetElement(j);
                    rgbColor |= static_cast<unsigned int>(channelValue * 255) << (j * 8);
                }
                memcpy(pointBeginPointer, &rgbColor, sizeof(uint32_t));
                return pointBeginPointer;
            } };
        };

        pointFillers.emplace_back(HandleXYZ());
        if (auto pointFiller = HandleEntityId())
        {
            pointFillers.emplace_back(pointFiller.value());
        }
        if (auto pointFiller = HandleClassAndClassColor())
        {
            pointFillers.emplace_back(pointFiller.value());
        }

        const auto sizeInBytes = pointCount * message.point_step;
        message.data.resize(sizeInBytes);
        message.row_step = message.width * message.point_step;
        AZ_Assert(message.row_step * message.height == sizeInBytes, "Inconsistency in the size of point cloud data");

        parallel_for(
            static_cast<size_t>(0),
            pointCount,
            [&message,&pointFillers](size_t i)
            {
                unsigned char* start = message.data.data() + static_cast<size_t>(message.point_step * i);
                for (auto& filler : pointFillers)
                {
                    start = filler(start, i);
                }
            },
            AZ::simple_partitioner(1 << 18));

        m_pointCloudPublisher->publish(message);

        if (m_segmentationClassesPublisher)
        {
            vision_msgs::msg::LabelInfo segmentationClasses;
            for (const auto& segmentation_class : m_lidarCore.m_lidarConfiguration.m_segmentationClasses)
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
