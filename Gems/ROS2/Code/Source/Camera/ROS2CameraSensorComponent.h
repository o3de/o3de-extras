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

#include <AzCore/Component/Component.h>

#include "CameraSensor.h"
#include "CameraSensorConfiguration.h"
#include "ROS2/Communication/TopicConfiguration.h"
#include <AzCore/std/containers/vector.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    namespace CameraConstants
    {
        inline constexpr char ImageMessageType[] = "sensor_msgs::msg::Image";
        inline constexpr char DepthImageConfig[] = "Depth Image";
        inline constexpr char ColorImageConfig[] = "Color Image";
        inline constexpr char DepthInfoConfig[] = "Depth Camera Info";
        inline constexpr char ColorInfoConfig[] = "Color Camera Info";
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
        ROS2CameraSensorComponent(const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration);

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

        template<typename CameraType>
        AZStd::vector<TopicConfiguration> GetCameraTopicConfiguration()
        {
            TopicConfiguration defaultConfig;
            return { defaultConfig };
        }

        template<typename CameraType>
        AZStd::vector<TopicConfiguration> GetCameraInfoTopicConfiguration()
        {
            TopicConfiguration defaultConfig;
            return { defaultConfig };
        }

        template<>
        AZStd::vector<TopicConfiguration> GetCameraTopicConfiguration<CameraColorSensor>()
        {
            return { m_sensorConfiguration.m_publishersConfigurations[CameraConstants::ColorImageConfig] };
        }

        template<>
        AZStd::vector<TopicConfiguration> GetCameraInfoTopicConfiguration<CameraColorSensor>()
        {
            return { m_sensorConfiguration.m_publishersConfigurations[CameraConstants::ColorInfoConfig] };
        }

        template<>
        AZStd::vector<TopicConfiguration> GetCameraTopicConfiguration<CameraDepthSensor>()
        {
            return { m_sensorConfiguration.m_publishersConfigurations[CameraConstants::DepthImageConfig] };
        }

        template<>
        AZStd::vector<TopicConfiguration> GetCameraInfoTopicConfiguration<CameraDepthSensor>()
        {
            return { m_sensorConfiguration.m_publishersConfigurations[CameraConstants::DepthInfoConfig] };
        }

        template<>
        AZStd::vector<TopicConfiguration> GetCameraTopicConfiguration<CameraRGBDSensor>()
        {
            // Note: the order matters.
            return { m_sensorConfiguration.m_publishersConfigurations[CameraConstants::ColorImageConfig],
                     m_sensorConfiguration.m_publishersConfigurations[CameraConstants::DepthImageConfig] };
        }

        template<>
        AZStd::vector<TopicConfiguration> GetCameraInfoTopicConfiguration<CameraRGBDSensor>()
        {
            // Note: the order matters.
            return { m_sensorConfiguration.m_publishersConfigurations[CameraConstants::ColorInfoConfig],
                     m_sensorConfiguration.m_publishersConfigurations[CameraConstants::DepthInfoConfig] };
        }

        //! Helper that adds publishers based on predefined configuration.
        template<typename PublishedData>
        void AddPublishersFromConfiguration(
            const AZStd::vector<TopicConfiguration> configurations,
            AZStd::vector<std::shared_ptr<rclcpp::Publisher<PublishedData>>>& publishers)
        {
            for (const auto& configuration : configurations)
            {
                AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), configuration.m_topic);
                auto ros2Node = ROS2Interface::Get()->GetNode();
                auto publisher = ros2Node->create_publisher<PublishedData>(fullTopic.data(), configuration.GetQoS());
                publishers.emplace_back(publisher);
            }
        }

        //! Helper that adds an image source.
        //! @tparam CameraType type of camera sensor (eg 'CameraColorSensor')
        template<typename CameraType>
        void AddImageSource()
        {
            CameraConfiguration cameraConfiguration = { m_cameraConfiguration.m_verticalFieldOfViewDeg,
                                                        m_cameraConfiguration.m_width,
                                                        m_cameraConfiguration.m_height };
            const CameraSensorDescription description{ GetCameraNameFromFrame(GetEntity()), cameraConfiguration };
            const auto cameraImagePublisherConfigs = GetCameraTopicConfiguration<CameraType>();
            AddPublishersFromConfiguration(cameraImagePublisherConfigs, m_imagePublishers);
            const auto cameraInfoPublisherConfigs = GetCameraInfoTopicConfiguration<CameraType>();
            AddPublishersFromConfiguration(cameraInfoPublisherConfigs, m_cameraInfoPublishers);
            m_cameraSensor = AZStd::make_shared<CameraType>(description, GetEntityId());
        }

        //! Retrieve camera name from ROS2FrameComponent's FrameID.
        //! @param entity pointer entity that has ROS2FrameComponent
        //! @returns FrameID from ROS2FrameComponent
        AZStd::string GetCameraNameFromFrame(const AZ::Entity* entity) const;

        void FrequencyTick() override;

        CameraSensorConfiguration m_cameraConfiguration;
        AZStd::string m_frameName;

        AZStd::vector<ImagePublisherPtrType> m_imagePublishers;
        AZStd::shared_ptr<CameraSensor> m_cameraSensor;
        AZStd::vector<CameraInfoPublisherPtrType> m_cameraInfoPublishers;
    };
} // namespace ROS2
