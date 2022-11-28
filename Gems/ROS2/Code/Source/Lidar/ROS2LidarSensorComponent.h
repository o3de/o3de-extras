/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarRaycaster.h"
#include "LidarTemplate.h"
#include "LidarTemplateUtils.h"
#include <ROS2/Lidar/LidarRaycasterBus.h>
#include <ROS2/Lidar/LidarRegistrarBus.h>
#include <ROS2/Lidar/LidarSystemBus.h>
#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ROS2
{
    //! Lidar sensor Component.
    //! Lidars (Light Detection and Ranging) emit laser light and measure it after reflection.
    //! Lidar Component allows customization of lidar type and behavior and encapsulates both simulation
    //! and data publishing. It requires ROS2FrameComponent.
    class ROS2LidarSensorComponent : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, "{502A955F-7742-4E23-AD77-5E4063739DCA}", ROS2SensorComponent);
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

        AZ::Crc32 OnLidarModelSelected();
        AZ::Crc32 OnLidarImplementationSelected();
        void FetchLidarImplementationFeatures();
        bool IsConfigurationVisible() const;
        bool IsIgnoredLayerConfigurationVisible() const;
        bool IsEntityExclusionVisible() const;
        bool IsMaxPointsConfigurationVisible() const;
        AZStd::vector<AZStd::string> GetLidarSystemList();
        void ConnectToLidarRaycaster();
        void ConfigureLidarRaycaster();

        LidarSystemFeatures m_lidarSystemFeatures;
        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::LidarModel::Custom3DLidar;
        LidarTemplate m_lidarParameters = LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel::Custom3DLidar);
        AZStd::vector<AZ::Vector3> m_lastRotations;

        AZStd::string m_lidarSystem;
        // A structure that maps each lidar implementation busId to the busId of a raycaster created by this LidarSensorComponent.
        AZStd::unordered_map<AZStd::string, AZ::Uuid> m_implementationToRaycasterMap;
        AZ::Uuid m_lidarRaycasterUuid;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;

        // Used only when visualisation is on - points differ since they are in global transform as opposed to local
        AZStd::vector<AZ::Vector3> m_visualisationPoints;
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        AZStd::vector<AZ::Vector3> m_lastScanResults;

        unsigned int m_ignoredLayerIndex = 0;
        bool m_ignoreLayer = false;
        AZStd::vector<AZ::EntityId> m_excludedEntities;

        bool m_addPointsAtMax = false;
    };
} // namespace ROS2
