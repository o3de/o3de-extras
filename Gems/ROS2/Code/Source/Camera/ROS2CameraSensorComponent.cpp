/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Camera/ROS2CameraSensorComponent.h"
#include "Frame/ROS2FrameComponent.h"
#include "ROS2/ROS2Bus.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Utilities/ROS2Names.h>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace ROS2
{
    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
        const char* kCameraInfoMessageType = "sensor_msgs::msg::CameraInfo";

        AZStd::pair<AZStd::string, PublisherConfiguration> MakePublisherConfigurationPair(
            const AZStd::string& topic, const AZStd::string& messageType)
        {
            PublisherConfiguration config;
            config.m_topic = topic;
            config.m_type = messageType;
            return AZStd::make_pair(messageType, config);
        }
    } // namespace Internal

    ROS2CameraSensorComponent::ROS2CameraSensorComponent()
    {
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakePublisherConfigurationPair("camera_image", Internal::kImageMessageType));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakePublisherConfigurationPair("camera_info", Internal::kCameraInfoMessageType));
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()
                ->Version(1)
                ->Field("CameraName", &ROS2CameraSensorComponent::m_cameraName)
                ->Field("VerticalFieldOfViewDeg", &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg)
                ->Field("Width", &ROS2CameraSensorComponent::m_width)
                ->Field("Height", &ROS2CameraSensorComponent::m_height);

            AZ::EditContext* ec = serialize->GetEditContext();
            if (ec)
            {
                ec->Class<ROS2CameraSensorComponent>("ROS2 Camera Sensor", "[Camera component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_cameraName, "Camera Name", "This is the camera name.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg,
                        "Vertical field of view",
                        "Camera's vertical (y axis) field of view in degrees.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_width, "Image width", "Image width")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_height, "Image height", "Image height");
            }
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();

        const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImageMessageType];
        AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
        m_imagePublisher =
            ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());

        const auto cameraInfoPublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kCameraInfoMessageType];
        AZStd::string cameraInfoFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraInfoPublisherConfig.m_topic);
        AZ_TracePrintf("ROS2", "Creating publisher for camera info on topic %s", cameraInfoFullTopic.data());
        m_cameraInfoPublisher =
            ros2Node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoFullTopic.data(), cameraInfoPublisherConfig.GetQoS());

        m_cameraSensor.emplace(CameraSensorDescription{ m_cameraName, m_VerticalFieldOfViewDeg, m_width, m_height });
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensor.reset();
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        if (!m_cameraSensor)
        {
            return;
        }
        m_cameraSensor->RequestFrame(
            transform,
            [this](const AZ::RPI::AttachmentReadback::ReadbackResult& result)
            {
                const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;

                AZStd::string frameName = GetEntity()->FindComponent<ROS2FrameComponent>()->GetFrameID();
                sensor_msgs::msg::Image message;
                message.encoding = sensor_msgs::image_encodings::RGBA8;

                message.width = descriptor.m_size.m_width;
                message.height = descriptor.m_size.m_height;
                message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
                message.header.frame_id = frameName.c_str();

                m_imagePublisher->publish(message);

                sensor_msgs::msg::CameraInfo cameraInfo;
                cameraInfo.header.frame_id = frameName.c_str();
                cameraInfo.width = descriptor.m_size.m_width;
                cameraInfo.height = descriptor.m_size.m_height;
                cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                cameraInfo.k = m_cameraSensor->GetCameraSensorDescription().m_cameraIntrinsics;
                m_cameraInfoPublisher->publish(cameraInfo);
            });
    }
} // namespace ROS2
