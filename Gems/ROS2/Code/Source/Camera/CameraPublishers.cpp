/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraConstants.h"
#include "CameraPublishers.h"
#include "CameraSensor.h"
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <ROS2/ROS2Bus.h>

namespace ROS2
{
    namespace Internal
    {
        using TopicConfigurations = AZStd::unordered_map<CameraSensorDescription::CameraChannelType, TopicConfiguration>;

        template<typename CameraType>
        TopicConfigurations GetCameraTopicConfiguration([[maybe_unused]] const SensorConfiguration& sensorConfiguration)
        {
            AZ_Error("GetCameraTopicConfiguration", false, "Invalid camera template type!");
            return TopicConfigurations();
        }

        template<typename CameraType>
        TopicConfigurations GetCameraInfoTopicConfiguration([[maybe_unused]] const SensorConfiguration& sensorConfiguration)
        {
            AZ_Error("GetCameraInfoTopicConfiguration", false, "Invalid camera template type!");
            return TopicConfigurations();
        }

        template<>
        TopicConfigurations GetCameraTopicConfiguration<CameraColorSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::RGB, sensorConfiguration.m_publishersConfigurations[CameraConstants::ColorImageConfig] } };
        }

        template<>
        TopicConfigurations GetCameraInfoTopicConfiguration<CameraColorSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::RGB, sensorConfiguration.m_publishersConfigurations[CameraConstants::ColorInfoConfig] } };
        }

        template<>
        TopicConfigurations GetCameraTopicConfiguration<CameraDepthSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::DEPTH, sensorConfiguration.m_publishersConfigurations[CameraConstants::DepthImageConfig] } };
        }

        template<>
        TopicConfigurations GetCameraInfoTopicConfiguration<CameraDepthSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraPublishers::DEPTH, sensorConfiguration.m_publishersConfigurations[CameraConstants::DepthInfoConfig] } };
        }

        //! Helper that adds publishers based on predefined configuration.
        template<typename PublishedData>
        void AddPublishersFromConfiguration(
            const TopicConfigurations configurations,
            AZStd::unordered_map<CameraSensorDescription::CameraChannelType, std::shared_ptr<rclcpp::Publisher<PublishedData>>>& publishers)
        {
            for (const auto& [channel, configuration] : configurations)
            {
                AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), configuration.m_topic);
                auto ros2Node = ROS2Interface::Get()->GetNode();
                auto publisher = ros2Node->create_publisher<PublishedData>(fullTopic.data(), configuration.GetQoS());
                publishers[channel] = publisher;
            }
        }

        //! Helper that adds publishers for a camera type.
        //! @tparam CameraType type of camera sensor (eg 'CameraColorSensor')
        template<typename CameraType>
        void AddCameraPublishers(
            const SensorConfiguration& sensorConfiguration,
            AZStd::unordered_map<CameraSensorDescription::CameraChannelType, ImagePublisherPtrType>& imagePublishers,
            AZStd::unordered_map<CameraSensorDescription::CameraChannelType, InfoPublisherPtrType>& infoPublishers)
        {
            const auto cameraImagePublisherConfigs = GetCameraTopicConfiguration<CameraType>(sensorConfiguration);
            AddPublishersFromConfiguration(cameraImagePublisherConfigs, imagePublishers);
            const auto cameraInfoPublisherConfigs = GetCameraInfoTopicConfiguration<CameraType>(sensorConfiguration);
            AddPublishersFromConfiguration(cameraInfoPublisherConfigs, cameraInfoPublishers);
        }
    } // namespace Internal

    CameraPublishers::CameraPublishers(const CameraSensorDescription& cameraDescription)
    {
        if (cameraDescription.m_cameraConfiguration.m_colorCamera)
        {
            Internal::AddCameraPublishers<CameraColorSensor>(cameraDescription.m_sensorConfiguration, m_imagePublishers, m_infoPublishers);
        }

        if (cameraDescription.m_cameraConfiguration.m_depthCamera)
        {
            Internal::AddCameraPublishers<CameraDepthSensor>(cameraDescription.m_sensorConfiguration, m_imagePublishers, m_infoPublishers);
        }
    }

    ImagePublisherPtrType CameraPublishers::GetImagePublisher(ChannelType type)
    {
        AZ_Error("GetImagePublisher", m_imagePublishers.count(type) != 1, "No publisher of this type, logic error!");
        return m_imagePublishers.at(type);
    }

    CameraInfoPublisherPtrType CameraPublishers::GetInfoPublisher(ChannelType type)
    {
        AZ_Error("GetInfoPublisher", m_imagePublishers.count(type) != 1, "No publisher of this type, logic error!");
        return m_infoPublishers.at(type);
    }
} // namespace ROS2
