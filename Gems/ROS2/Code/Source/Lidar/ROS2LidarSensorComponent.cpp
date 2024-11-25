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
#include <ROS2/Lidar/ClassSegmentationBus.h>
#include <ROS2/Lidar/PC2PostProcessingBus.h>
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
        FieldFormat::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2LidarSensorComponent, SensorBaseType>()
                ->Version(3)
                ->Field("lidarCore", &ROS2LidarSensorComponent::m_lidarCore)
                ->Field("messageFormat", &ROS2LidarSensorComponent::m_messageFormat)
                ->Field("pointCloudIsDense", &ROS2LidarSensorComponent::m_pointcloudIsDense)
                ->Field("pointCloudOrdering", &ROS2LidarSensorComponent::m_pointcloudOrderingEnabled);

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
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2LidarSensorComponent::m_messageFormat, "Point Cloud 2 message format", "")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ROS2LidarSensorComponent::OnMessageFormatChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2LidarSensorComponent::m_pointcloudIsDense,
                        "Dense pointcloud",
                        "If enabled, only the points that hit an obstacle are processed and published. Having this option enabled improves "
                        "performance but disallows pointcloud ordering and points at max.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ROS2LidarSensorComponent::OnDensePointcloudChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2LidarSensorComponent::m_pointcloudOrderingEnabled,
                        "Pointcloud ordering",
                        "Message's width and height match those of the used ray pattern. Only available for sparse (non-dense) "
                        "pointclouds.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2LidarSensorComponent::IsPointcloudOrderingVisible);
            }
        }
    }

    void ROS2LidarSensorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE( "ROS2LidarSensor" ));
    }

    void ROS2LidarSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2LidarSensorComponent::Init()
    {
        if (m_messageFormat.empty())
        {
            AZ_Warning("ROS2", false, "No message format provided. Applying default message format.");
            m_messageFormat = GetDefaultMessageFormat();
        }
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

        m_pointCloudMessageWriter = PointCloudMessageWriter(m_messageFormat);
        m_lidarCore.Init(GetEntityId(), GetRequestResultFlags());

        m_lidarRaycasterId = m_lidarCore.GetLidarRaycasterId();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

        const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[PointCloudType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_pointCloudPublisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic.data(), publisherConfig.GetQoS());

        if (IsFlagEnabled(RaycastResultFlags::SegmentationData, m_lidarCore.GetResultFlags()))
        {
            m_segmentationClassesPublisher = ros2Node->create_publisher<vision_msgs::msg::LabelInfo>(
                ROS2Names::GetNamespacedName(GetNamespace(), "segmentation_classes").data(), publisherConfig.GetQoS());
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

    const Pc2MessageFormat& ROS2LidarSensorComponent::GetDefaultMessageFormat()
    {
        static const Pc2MessageFormat DefaultMessageFormat = {
            FieldFormat(FieldFlags::PositionXYZF32), FieldFormat(FieldFlags::Padding32),          FieldFormat(FieldFlags::RangeU32),
            FieldFormat(FieldFlags::IntensityF32),   FieldFormat(FieldFlags::SegmentationData96),
        };

        return DefaultMessageFormat;
    }

    void ROS2LidarSensorComponent::TransformToLidarLocalSpace(AZStd::span<AZ::Vector3> pointSpan) const
    {
        const auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>();
        const auto inverseLidarTM = entityTransform->GetWorldTM().GetInverse();
        for (auto pointIt = pointSpan.begin(); pointIt != pointSpan.end(); ++pointIt)
        {
            *pointIt = inverseLidarTM.TransformPoint(*pointIt);
        }
    }

    RaycastResultFlags ROS2LidarSensorComponent::GetRequestResultFlags() const
    {
        auto flags = GetNecessaryProviders(m_messageFormat) | RaycastResultFlags::Point; // We need points for visualisation.

        if (!m_pointcloudIsDense)
        {
            flags |= RaycastResultFlags::IsHit;
        }

        return flags;
    }

    bool ROS2LidarSensorComponent::IsPointcloudOrderingVisible() const
    {
        return !m_pointcloudIsDense;
    }

    void ROS2LidarSensorComponent::FrequencyTick()
    {
        AZStd::optional<RaycastResults> lastScanResults = m_lidarCore.PerformRaycast();
        if (!lastScanResults.has_value() || !m_sensorConfiguration.m_publishingEnabled)
        {
            return;
        }

        if (auto pointSpan = lastScanResults->GetFieldSpan<RaycastResultFlags::Point>(); pointSpan.has_value())
        {
            TransformToLidarLocalSpace(pointSpan.value());
        }

        const auto outcome = PublishRaycastResults(lastScanResults.value());
        if (!outcome.IsSuccess())
        {
            AZ_Error("ROS2", false, "Failed to publish raycast results: %s", outcome.GetError());
        }
    }

    template<typename T>
    using Pc2MsgIt = sensor_msgs::PointCloud2Iterator<T>;

    AZ::Outcome<void, const char*> ROS2LidarSensorComponent::PublishRaycastResults(const RaycastResults& results)
    {
        if (!m_pointCloudMessageWriter.has_value())
        {
            return AZ::Failure("Point cloud message writer was uninitialized.");
        }

        if (!results.IsCompliant(m_lidarCore.GetResultFlags()))
        {
            return AZ::Failure("Received raycast results that were not expected.");
        }

        const size_t rayLayerCount = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_layers;
        const size_t rayCountPerLayer = m_lidarCore.m_lidarConfiguration.m_lidarParameters.m_numberOfIncrements;
        if (m_pointcloudOrderingEnabled && results.GetCount() != rayLayerCount * rayCountPerLayer)
        {
            return AZ::Failure("Received raycast results with dimensions that were not expected.");
        }

        const size_t pcWidth = m_pointcloudOrderingEnabled ? rayCountPerLayer : results.GetCount();
        const size_t pcHeight = m_pointcloudOrderingEnabled ? rayLayerCount : 1U;
        m_pointCloudMessageWriter->Reset(
            GetEntity()->FindComponent<ROS2FrameComponent>()->GetFrameID(),
            ROS2Interface::Get()->GetROSTimestamp(),
            pcWidth,
            pcHeight,
            m_pointcloudIsDense);

        m_pointCloudMessageWriter->WriteResults(results, !m_pointcloudIsDense);

        PC2PostProcessingRequestBus::Event(GetEntityId(), &PC2PostProcessingRequests::ApplyPostProcessing, m_pointCloudMessageWriter->GetMessage());

        m_pointCloudPublisher->publish(m_pointCloudMessageWriter->GetMessage());

        return AZ::Success();
    }

    AZ::Crc32 ROS2LidarSensorComponent::OnMessageFormatChanged()
    {
        size_t offset = 0U;
        bool positionPresent = false;
        for (FieldFormat& fieldFormat : m_messageFormat)
        {
            if (fieldFormat.m_name.empty())
            {
                if (auto fieldName = GetDefaultFieldName(fieldFormat.m_fieldFlag); fieldName.has_value())
                {
                    fieldFormat.m_name = fieldName.value();
                }
            }

            if (fieldFormat.m_fieldFlag == FieldFlags::PositionXYZF32)
            {
                positionPresent = true;
            }

            fieldFormat.m_fieldOffset = offset;
            offset += GetFieldByteSize(fieldFormat.m_fieldFlag);
        }

        if (!positionPresent)
        {
            m_messageFormat.push_back(FieldFormat(FieldFlags::PositionXYZF32));
        }

        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::Crc32 ROS2LidarSensorComponent::OnDensePointcloudChanged()
    {
        if (m_pointcloudIsDense)
        {
            m_lidarCore.m_lidarConfiguration.m_addPointsAtMax = false;
            m_pointcloudOrderingEnabled = false;
        }

        // This is to ensure that visibility of pointcloud ordering is updated.
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }
} // namespace ROS2
