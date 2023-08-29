/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraPublishers.h"
#include "CameraConstants.h"
#include "CameraSensor.h"
#include <AzCore/Component/EntityId.h>
#include <ROS2/Communication/FlexiblePublisher.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <memory>

namespace ROS2
{
    namespace Internal
    {
        using TopicConfigurations = AZStd::unordered_map<CameraSensorDescription::CameraChannelType, TopicConfiguration>;

        TopicConfiguration GetTopicConfiguration(const SensorConfiguration& sensorConfiguration, const AZStd::string& key)
        {
            auto ipos = sensorConfiguration.m_publishersConfigurations.find(key);
            AZ_Assert(ipos != sensorConfiguration.m_publishersConfigurations.end(), "Missing key in topic configuration!");
            return ipos != sensorConfiguration.m_publishersConfigurations.end() ? ipos->second : TopicConfiguration{};
        }

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
            return { { CameraSensorDescription::CameraChannelType::RGB,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::ColorImageConfig) } };
        }

        template<>
        TopicConfigurations GetCameraInfoTopicConfiguration<CameraColorSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraSensorDescription::CameraChannelType::RGB,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::ColorInfoConfig) } };
        }

        template<>
        TopicConfigurations GetCameraTopicConfiguration<CameraDepthSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraSensorDescription::CameraChannelType::DEPTH,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::DepthImageConfig) } };
        }

        template<>
        TopicConfigurations GetCameraInfoTopicConfiguration<CameraDepthSensor>(const SensorConfiguration& sensorConfiguration)
        {
            return { { CameraSensorDescription::CameraChannelType::DEPTH,
                       GetTopicConfiguration(sensorConfiguration, CameraConstants::DepthInfoConfig) } };
        }

        //! Helper that adds publishers based on predefined configuration.
        template<typename PublishedData>
        void AddPublishersFromConfiguration(
            const AZStd::string& cameraNamespace,
            const TopicConfigurations configurations,
            AZStd::unordered_map<CameraSensorDescription::CameraChannelType, std::shared_ptr<FlexiblePublisher<PublishedData>>>& publishers,
            const AZ::EntityId& entityId)
        {
            for (const auto& [channel, configuration] : configurations)
            {
                auto publisher = std::make_shared<FlexiblePublisher<PublishedData>>(configuration, cameraNamespace, entityId, "Camera Sensor");
                publishers[channel] = publisher;
            }
        }

        //! Helper that adds publishers for a camera type.
        //! @tparam CameraType type of camera sensor (eg 'CameraColorSensor').
        //! @param cameraDescription complete information about camera configuration.
        //! @param imagePublishers publishers of raw image formats (color image, depth image, ..).
        //! @param infoPublishers publishers of camera_info messages for each image topic.
        template<typename CameraType>
        void AddCameraPublishers(
            const CameraSensorDescription& cameraDescription,
            AZStd::unordered_map<CameraSensorDescription::CameraChannelType, CameraPublishers::ImagePublisherPtrType>& imagePublishers,
            AZStd::unordered_map<CameraSensorDescription::CameraChannelType, CameraPublishers::CameraInfoPublisherPtrType>& infoPublishers,
            const AZ::EntityId& entityId)
        {
            const auto cameraImagePublisherConfigs = GetCameraTopicConfiguration<CameraType>(cameraDescription.m_sensorConfiguration);
            AddPublishersFromConfiguration(cameraDescription.m_cameraNamespace, cameraImagePublisherConfigs, imagePublishers, entityId);
            const auto cameraInfoPublisherConfigs = GetCameraInfoTopicConfiguration<CameraType>(cameraDescription.m_sensorConfiguration);
            AddPublishersFromConfiguration(cameraDescription.m_cameraNamespace, cameraInfoPublisherConfigs, infoPublishers, entityId);
        }
    } // namespace Internal

    CameraPublishers::CameraPublishers(const CameraSensorDescription& cameraDescription, const AZ::EntityId& entityId)
    {
        if (cameraDescription.m_cameraConfiguration.m_colorCamera)
        {
            Internal::AddCameraPublishers<CameraColorSensor>(cameraDescription, m_imagePublishers, m_infoPublishers, entityId);
        }

        if (cameraDescription.m_cameraConfiguration.m_depthCamera)
        {
            Internal::AddCameraPublishers<CameraDepthSensor>(cameraDescription, m_imagePublishers, m_infoPublishers, entityId);
        }
    }

    CameraPublishers::ImagePublisherPtrType CameraPublishers::GetImagePublisher(CameraSensorDescription::CameraChannelType type)
    {
        AZ_Error("GetImagePublisher", m_imagePublishers.count(type) == 1, "No publisher of this type, logic error!");
        return m_imagePublishers.at(type);
    }

    CameraPublishers::CameraInfoPublisherPtrType CameraPublishers::GetInfoPublisher(CameraSensorDescription::CameraChannelType type)
    {
        AZ_Error("GetInfoPublisher", m_infoPublishers.count(type) == 1, "No publisher of this type, logic error!");
        return m_infoPublishers.at(type);
    }
} // namespace ROS2
