/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <ROS2/Sensor/ROS2SensorComponent.h>

#include <AzCore/Component/Component.h>

#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>

#include "CameraSensor.h"

namespace ROS2
{
    namespace CameraConstants
    {
        inline constexpr char ImageMessageType[] = "sensor_msgs::msg::Image";
        inline constexpr char DepthImageConfig[] = "Depth Image";
        inline constexpr char ColorImageConfig[] = "Color Image";
        inline constexpr char InfoConfig[] = "Camera Info";
        inline constexpr char CameraInfoMessageType[] = "sensor_msgs::msg::CameraInfo";
    } // namespace CameraConstants

    //! ROS2 Camera sensor component class
    //! Allows turning an entity into a camera sensor
    //! Can be parametrized with following values:
    //!   - camera name
    //!   - camera image width and height in pixels
    //!   - camera vertical field of view in degrees
    //! Camera frustum is facing negative Z axis; image plane is parallel to X,Y plane: X - right, Y - up
    class ROS2CameraSensorComponent : public ROS2SensorComponent
    {
    public:
        ROS2CameraSensorComponent() = default;

        ROS2CameraSensorComponent(
            const SensorConfiguration& sensorConfiguration,
            float verticalFieldOfViewDeg,
            int width,
            int height,
            bool colorCamera,
            bool depthCamera);

        ~ROS2CameraSensorComponent() override = default;
        AZ_COMPONENT(ROS2CameraSensorComponent, "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}", ROS2SensorComponent);
        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    private:
        //! Pointer to ROS2 image publisher type
        using ImagePublisherPtrType = std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>;

        //! Pointer to ROS2 camera sensor publisher type
        using CameraInfoPublisherPtrType = std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>;

        //! Type that combines pointer to ROS2 publisher and CameraSensor
        using PublisherSensorPtrPair = AZStd::pair<ImagePublisherPtrType, AZStd::shared_ptr<CameraSensor>>;

        //! Helper to construct a PublisherSensorPtrPair with a pointer to ROS2 publisher and intrinsic calibration.
        //! @tparam CameraType type of camera sensor (eg 'CameraColorSensor')
        //! @param publisher pointer to ROS2 image publisher
        //! @param description CameraSensorDescription with intrinsic calibration
        //! @return PublisherSensorPtrPair with all provided parameters
        template<typename CameraType>
        PublisherSensorPtrPair CreatePair(ImagePublisherPtrType publisher, const CameraSensorDescription& description) const
        {
            return { publisher, AZStd::make_shared<CameraType>(description) };
        }

        //! Retrieve camera name from ROS2FrameComponent's FrameID.
        //! @param entity pointer entity that has ROS2FrameComponent
        //! @returns FrameID from ROS2FrameComponent
        AZStd::string GetCameraNameFromFrame(const AZ::Entity* entity) const;

        float m_verticalFieldOfViewDeg = 90.0f;
        int m_width = 640;
        int m_height = 480;
        bool m_colorCamera = true;
        bool m_depthCamera = true;
        AZStd::string m_frameName;

        void FrequencyTick() override;

        AZStd::vector<ImagePublisherPtrType> m_imagePublishers;
        AZStd::shared_ptr<CameraSensor> m_cameraSensor;
        CameraInfoPublisherPtrType m_cameraInfoPublisher;
    };
} // namespace ROS2
