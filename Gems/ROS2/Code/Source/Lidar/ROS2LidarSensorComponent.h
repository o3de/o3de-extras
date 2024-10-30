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
#include <ROS2/Lidar/LidarRegistrarBus.h>
#include <ROS2/Lidar/LidarSystemBus.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/label_info.hpp>

#include "LidarCore.h"
#include "LidarRaycaster.h"
#include "LidarSensorConfiguration.h"
#include "Publishing/PointCloudMessageWriter.h"

namespace ROS2
{
    //! Lidar sensor Component.
    //! Lidars (Light Detection and Ranging) emit laser light and measure it after reflection.
    //! Lidar Component allows customization of lidar type and behavior and encapsulates both simulation
    //! and data publishing. It requires ROS2FrameComponent.
    class ROS2LidarSensorComponent : public ROS2SensorComponentBase<TickBasedSource>
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, ROS2LidarSensorComponentTypeId, SensorBaseType);
        ROS2LidarSensorComponent();
        ROS2LidarSensorComponent(const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration);
        ~ROS2LidarSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        static const Pc2MessageFormat& GetDefaultMessageFormat();
        void TransformToLidarLocalSpace(AZStd::span<AZ::Vector3> pointSpan) const;
        RaycastResultFlags GetRequestResultFlags() const;
        void FrequencyTick();
        AZ::Outcome<void, const char*> PublishRaycastResults(const RaycastResults& results);
        AZ::Crc32 OnMessageFormatChanged();

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::LabelInfo>> m_segmentationClassesPublisher;

        AZStd::optional<PointCloudMessageWriter> m_pointCloudMessageWriter;
        Pc2MessageFormat m_messageFormat;

        LidarCore m_lidarCore;
        LidarId m_lidarRaycasterId;

        bool m_pointcloudOrderingEnabled{ false };
    };
} // namespace ROS2
