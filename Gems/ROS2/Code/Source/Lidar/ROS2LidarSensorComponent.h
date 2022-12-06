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
        //////////////////////////////////////////////////////////////////////////
        void SetPhysicsScene();

        AZ::Crc32 OnLidarModelSelected();
        bool IsConfigurationVisible() const;

        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::LidarModel::Generic3DLidar;
        LidarTemplate m_lidarParameters = LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel::Generic3DLidar);
        LidarRaycaster m_lidarRaycaster;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;

        // Used only when visualisation is on - points differ since they are in global transform as opposed to local
        AZStd::vector<AZ::Vector3> m_visualisationPoints;
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        AZStd::vector<AZ::Vector3> m_lastScanResults;

        unsigned int m_ignoredLayerIndex = 0;
        bool m_ignoreLayer = false;
    };
} // namespace ROS2
