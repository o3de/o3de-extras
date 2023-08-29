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
#include <ROS2/Communication/FlexiblePublisher.h>
#include <ROS2/Lidar/LidarRegistrarBus.h>
#include <ROS2/Lidar/LidarSystemBus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "LidarRaycaster.h"
#include "LidarSensorConfiguration.h"

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
        ROS2LidarSensorComponent(const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration);
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
        void Visualize() override;

        void ConnectToLidarRaycaster();
        void ConfigureLidarRaycaster();

        // A structure that maps each lidar implementation busId to the busId of a raycaster created by this LidarSensorComponent.
        AZStd::unordered_map<AZStd::string, LidarId> m_implementationToRaycasterMap;
        bool m_canRaycasterPublish = false;
        LidarId m_lidarRaycasterId;
        std::shared_ptr<FlexiblePublisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;

        // Used only when visualization is on - points differ since they are in global transform as opposed to local
        AZStd::vector<AZ::Vector3> m_visualizationPoints;
        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        LidarSensorConfiguration m_lidarConfiguration;

        AZStd::vector<AZ::Vector3> m_lastRotations;
        RaycastResult m_lastScanResults;
    };
} // namespace ROS2
