/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Lidar/LidarRaycaster.h>
#include <Lidar/LidarTemplate.h>
#include <Lidar/LidarTemplateUtils.h>
#include <ROS2/Lidar/LidarRegistrarBus.h>
#include <ROS2/Lidar/LidarSystemBus.h>
#include <ROS2/Sensor/ROS2SensorTickableComponent.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ROS2
{
    //! Lidar sensor Component.
    //! Lidars (Light Detection and Ranging) emit laser light and measure it after reflection.
    //! Lidar Component allows customization of lidar type and behavior and encapsulates both simulation
    //! and data publishing. It requires ROS2FrameComponent.
    class ROS2LidarSensorComponent : public ROS2SensorTickableComponent
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, "{502A955F-7742-4E23-AD77-5E4063739DCA}", ROS2SensorTickableComponent);
        ROS2LidarSensorComponent();
        ~ROS2LidarSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        // ROS2SensorComponent overrides
        void FrequencyTick() override;
        void Visualise() override;

        bool IsConfigurationVisible() const;
        bool IsIgnoredLayerConfigurationVisible() const;
        bool IsEntityExclusionVisible() const;
        bool IsMaxPointsConfigurationVisible() const;

        AZ::Crc32 OnLidarModelSelected();
        AZ::Crc32 OnLidarImplementationSelected();
        void FetchLidarImplementationFeatures();
        AZStd::vector<AZStd::string> FetchLidarSystemList();
        void ConnectToLidarRaycaster();
        void ConfigureLidarRaycaster();

        LidarSystemFeatures m_lidarSystemFeatures;
        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::LidarModel::Custom3DLidar;
        LidarTemplate m_lidarParameters = LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel::Custom3DLidar);
        AZStd::vector<AZ::Vector3> m_lastRotations;

        AZStd::string m_lidarSystem;
        // A structure that maps each lidar implementation busId to the busId of a raycaster created by this LidarSensorComponent.
        AZStd::unordered_map<AZStd::string, LidarId> m_implementationToRaycasterMap;
        LidarId m_lidarRaycasterId;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;

        // Used only when visualisation is on - points differ since they are in global transform as opposed to local
        AZStd::vector<AZ::Vector3> m_visualisationPoints;
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        AZStd::vector<AZ::Vector3> m_lastScanResults;

        AZ::u32 m_ignoredLayerIndex = 0;
        bool m_ignoreLayer = false;
        AZStd::vector<AZ::EntityId> m_excludedEntities;

        bool m_addPointsAtMax = false;
    };
} // namespace ROS2
