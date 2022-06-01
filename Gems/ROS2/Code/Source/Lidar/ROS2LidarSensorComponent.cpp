/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Frame/ROS2FrameComponent.h"
#include "Lidar/ROS2LidarSensorComponent.h"
#include "Lidar/LidarTemplateUtils.h"
#include "ROS2/ROS2Bus.h"
#include "Utilities/ROS2Names.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/Scene.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kPointCloudType = "sensor_msgs::msg::PointCloud2";
    }

    void ROS2LidarSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        LidarTemplate::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2LidarSensorComponent, ROS2SensorComponent>()
                ->Version(1)
                ->Field("lidarModel", &ROS2LidarSensorComponent::m_lidarModel)
                ->Field("selfCollisionEntityId", &ROS2LidarSensorComponent::m_selfColliderEntityId)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2LidarSensorComponent>("ROS2 Lidar Sensor", "Lidar sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &ROS2LidarSensorComponent::m_lidarModel, "Lidar Model", "Lidar model")
                        ->EnumAttribute(LidarTemplate::LidarModel::Generic3DLidar, "Generic Lidar")
                        // TODO - show lidar template field values (read only) - see Reflect for LidarTemplate
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ROS2LidarSensorComponent::m_selfColliderEntityId, "Self collision Entity",
                                  "Entity to be transparent for the lidar. If not set, use this entity")
                    ;
            }
        }
    }

    ROS2LidarSensorComponent::ROS2LidarSensorComponent()
    {
        PublisherConfiguration pc;
        AZStd::string type = Internal::kPointCloudType;
        pc.m_type = type;
        pc.m_topic = "pc";
        m_sensorConfiguration.m_frequency = 10; // TODO - dependent on lidar type
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
    }

    void ROS2LidarSensorComponent::Visualise()
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

    void ROS2LidarSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

        const PublisherConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kPointCloudType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_pointCloudPublisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic.data(), publisherConfig.GetQoS());

        if (m_sensorConfiguration.m_visualise)
        {
            auto defaultScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(defaultScene);
        }

        if (m_selfColliderEntityId.IsValid())
        {
            m_lidarRaycaster.setSelfColliderEntity(m_selfColliderEntityId);
        } 
        else 
        {
            m_lidarRaycaster.setSelfColliderEntity(GetEntityId());
        }
    }

    void ROS2LidarSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_pointCloudPublisher.reset();
    }

    void ROS2LidarSensorComponent::FrequencyTick()
    {
        float distance = LidarTemplateUtils::GetTemplate(m_lidarModel).m_maxRange;
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>(); // TODO - go through ROS2Frame
        const auto directions = LidarTemplateUtils::PopulateRayDirections(m_lidarModel,
                                                                          entityTransform->GetWorldTM().GetEulerRadians());
        AZ::Vector3 start = entityTransform->GetWorldTM().GetTranslation();
        start.SetZ(start.GetZ());

        m_lastScanResults = m_lidarRaycaster.PerformRaycast(start, directions, distance);
        if (m_lastScanResults.empty())
        {
            AZ_TracePrintf("Lidar Sensor Component", "No results from raycast\n");
            return;
        }

        if (m_sensorConfiguration.m_visualise)
        {   // Store points for visualisation purposes, before transformations occur
            m_visualisationPoints = m_lastScanResults;
        }

        auto ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        auto message = sensor_msgs::msg::PointCloud2();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.height = 1;
        message.width = m_lastScanResults.size();
        message.point_step = sizeof(AZ::Vector3); // TODO - Point Fields can be custom
        message.row_step = message.width * message.point_step;

        // TODO - a list of supported fields should be returned by lidar implementation
        std::vector<std::string> point_field_names = { "x", "y", "z"};
        for (int i = 0; i < point_field_names.size(); i++)
        {   // TODO - placeholder impl
            sensor_msgs::msg::PointField pf;
            pf.name = point_field_names[i];
            pf.offset = i * 4;
            pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
            pf.count = 1;
            message.fields.push_back(pf);
        }

        auto lidarTM = entityTransform->GetWorldTM();
        lidarTM.Invert();

        // TODO - improve performance
        for (auto& point : m_lastScanResults)
        {
            point = lidarTM.TransformPoint(point);
        }

        size_t sizeInBytes = m_lastScanResults.size() * sizeof(AZ::Vector3);
        message.data.resize(sizeInBytes);
        AZ_Assert(message.row_step * message.height == sizeInBytes, "Inconsistency in the size of point cloud data");
        memcpy(message.data.data(), m_lastScanResults.data(), sizeInBytes);
        m_pointCloudPublisher->publish(message);
    }
} // namespace ROS2
