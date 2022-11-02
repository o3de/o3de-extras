/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Lidar/ROS2LidarSensorComponent.h"
#include "Lidar/LidarTemplateUtils.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"
#include "ROS2/Utilities/ROS2Names.h"
#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/RPISystemInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/PhysicsSystem.h>

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
                ->Field("LidarParameters", &ROS2LidarSensorComponent::m_lidarParameters)
                ->Field("IgnoreLayer", &ROS2LidarSensorComponent::m_ignoreLayer)
                ->Field("IgnoredLayerIndex", &ROS2LidarSensorComponent::m_ignoredLayerIndex);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2LidarSensorComponent>("ROS2 Lidar Sensor", "Lidar sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &ROS2LidarSensorComponent::m_lidarModel, "Lidar Model", "Lidar model")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &ROS2LidarSensorComponent::OnLidarModelSelected)
                    ->EnumAttribute(LidarTemplate::LidarModel::Generic3DLidar, "Generic Lidar")
                    // TODO - show lidar template field values (read only) - see Reflect for LidarTemplate
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ROS2LidarSensorComponent::m_lidarParameters,
                        "Lidar parameters",
                        "Configuration of Generic lidar")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &ROS2LidarSensorComponent::IsConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ROS2LidarSensorComponent::m_ignoreLayer,
                        "Ignore layer",
                        "Should we ignore selected layer index")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2LidarSensorComponent::m_ignoredLayerIndex,
                        "Ignored layer index",
                        "Layer index to ignore");
            }
        }
    }

    bool ROS2LidarSensorComponent::IsConfigurationVisible() const
    {
        return m_lidarModel == LidarTemplate::Generic3DLidar;
    }

    AZ::Crc32 ROS2LidarSensorComponent::OnLidarModelSelected()
    {
        m_lidarParameters = LidarTemplateUtils::GetTemplate(m_lidarModel);
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    ROS2LidarSensorComponent::ROS2LidarSensorComponent()
    {
        TopicConfiguration pc;
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

    void ROS2LidarSensorComponent::SetPhysicsScene()
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        auto foundBody = physicsSystem->FindAttachedBodyHandleFromEntityId(GetEntityId());
        auto lidarPhysicsSceneHandle = foundBody.first;
        if (foundBody.first == AzPhysics::InvalidSceneHandle)
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            lidarPhysicsSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        }

        AZ_Assert(lidarPhysicsSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid physics scene handle for entity");
        m_lidarRaycaster.SetRaycasterScene(lidarPhysicsSceneHandle);
    }

    void ROS2LidarSensorComponent::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for lidar sensor");

        const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kPointCloudType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_pointCloudPublisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic.data(), publisherConfig.GetQoS());

        SetPhysicsScene();
        if (m_sensorConfiguration.m_visualise)
        {
            auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        }
        m_lidarRaycaster.SetAddPointsMaxRange(m_lidarParameters.m_addPointsAtMax);
        ROS2SensorComponent::Activate();
    }

    void ROS2LidarSensorComponent::Deactivate()
    {
        ROS2SensorComponent::Deactivate();
        m_pointCloudPublisher.reset();
    }

    void ROS2LidarSensorComponent::FrequencyTick()
    {
        float distance = m_lidarParameters.m_maxRange;
        auto entityTransform = GetEntity()->FindComponent<AzFramework::TransformComponent>(); // TODO - go through ROS2Frame
        const auto directions =
            LidarTemplateUtils::PopulateRayDirections(m_lidarParameters, entityTransform->GetWorldTM().GetEulerRadians());
        AZ::Vector3 start = entityTransform->GetWorldTM().GetTranslation();
        start.SetZ(start.GetZ());

        auto worldToLidarTransform = entityTransform->GetWorldTM();
        worldToLidarTransform.Invert();

        m_lastScanResults =
            m_lidarRaycaster.PerformRaycast(start, directions, worldToLidarTransform, distance, m_ignoreLayer, m_ignoredLayerIndex);
        if (m_lastScanResults.empty())
        {
            AZ_TracePrintf("Lidar Sensor Component", "No results from raycast\n");
            return;
        }

        if (m_sensorConfiguration.m_visualise)
        { // Store points for visualisation purposes, in global frame
            auto localToWorldTM = entityTransform->GetWorldTM();

            // TODO - improve performance
            m_visualisationPoints = m_lastScanResults;
            for (auto& point : m_visualisationPoints)
            {
                point = localToWorldTM.TransformPoint(point);
            }
        }

        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        auto message = sensor_msgs::msg::PointCloud2();
        message.header.frame_id = ros2Frame->GetFrameID().data();
        message.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        message.height = 1;
        message.width = m_lastScanResults.size();
        message.point_step = sizeof(AZ::Vector3); // TODO - Point Fields can be custom
        message.row_step = message.width * message.point_step;

        // TODO - a list of supported fields should be returned by lidar implementation
        std::vector<std::string> point_field_names = { "x", "y", "z" };
        for (int i = 0; i < point_field_names.size(); i++)
        { // TODO - placeholder impl
            sensor_msgs::msg::PointField pf;
            pf.name = point_field_names[i];
            pf.offset = i * 4;
            pf.datatype = sensor_msgs::msg::PointField::FLOAT32;
            pf.count = 1;
            message.fields.push_back(pf);
        }

        size_t sizeInBytes = m_lastScanResults.size() * sizeof(AZ::Vector3);
        message.data.resize(sizeInBytes);
        AZ_Assert(message.row_step * message.height == sizeInBytes, "Inconsistency in the size of point cloud data");
        memcpy(message.data.data(), m_lastScanResults.data(), sizeInBytes);
        m_pointCloudPublisher->publish(message);
    }
} // namespace ROS2
