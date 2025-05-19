/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarCore.h"
#include "LidarRaycaster.h"
#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2Sensors/Lidar/LidarConfigurationRequestBus.h>
#include <ROS2Sensors/Lidar/LidarSensorConfiguration.h>
#include <ROS2Sensors/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/label_info.hpp>

namespace ROS2Sensors
{
    //! Lidar sensor Component.
    //! Lidars (Light Detection and Ranging) emit laser light and measure it after reflection.
    //! Lidar Component allows customization of lidar type and behavior and encapsulates both simulation
    //! and data publishing. It requires ROS2FrameComponent.
    class ROS2LidarSensorComponent
        : public ROS2SensorComponentBase<ROS2::TickBasedSource>
        , protected LidarConfigurationRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2LidarSensorComponent, ROS2Sensors::ROS2LidarSensorComponentTypeId, SensorBaseType);
        ROS2LidarSensorComponent();
        ROS2LidarSensorComponent(const SensorConfiguration& sensorConfiguration, const LidarSensorConfiguration& lidarConfiguration);
        ~ROS2LidarSensorComponent() = default;
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        // LidarConfigurationRequestBus::Handler overrides
        const LidarSensorConfiguration GetConfiguration() const override;
        void SetConfiguration(const LidarSensorConfiguration& configuration) override;
        AZStd::string GetModelName() const override;
        void SetModelName(const AZStd::string& name) override;
        bool IsSegmentationEnabled() const override;
        void SetSegmentationEnabled(bool enabled) override;
        bool IsAddPointsAtMaxEnabled() const override;
        void SetAddPointsAtMaxEnabled(bool addPoints) override;
        bool Is2D() const override;
        float GetMinHAngle() const override;
        void SetMinHAngle(float angle) override;
        float GetMaxHAngle() const override;
        void SetMaxHAngle(float angle) override;
        float GetMinVAngle() const override;
        void SetMinVAngle(float angle) override;
        float GetMaxVAngle() const override;
        void SetMaxVAngle(float angle) override;
        unsigned int GetLayers() const override;
        void SetLayers(unsigned int layers) override;
        unsigned int GetNumberOfIncrements() const override;
        void SetNumberOfIncrements(unsigned int increments) override;
        float GetMinRange() const override;
        void SetMinRange(float range) override;
        float GetMaxRange() const override;
        void SetMaxRange(float range) override;
        const LidarTemplate::NoiseParameters& GetNoiseParameters() const override;
        void SetNoiseParameters(const LidarTemplate::NoiseParameters& params) override;
        //////////////////////////////////////////////////////////////////////////

        void FrequencyTick();
        void PublishRaycastResults(const RaycastResults& results);

        bool m_canRaycasterPublish = false;
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> m_pointCloudPublisher;
        std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::LabelInfo>> m_segmentationClassesPublisher;

        LidarCore m_lidarCore;

        LidarId m_lidarRaycasterId;
    };
} // namespace ROS2Sensors
