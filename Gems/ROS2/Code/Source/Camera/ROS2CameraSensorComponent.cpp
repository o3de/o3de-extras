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

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>

#include <sensor_msgs/image_encodings.hpp>
#include <Utilities/ROS2Names.h>

namespace ROS2
{
    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
    }

    ROS2CameraSensorComponent::ROS2CameraSensorComponent() {
        PublisherConfiguration pc;
        auto type = Internal::kImageMessageType;
        pc.m_type = type;
        pc.m_topic = "camera_image";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, pc));
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
            if (ec) {
                ec->Class<ROS2CameraSensorComponent>("ROS2 Camera Sensor", "[Camera component]")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_cameraName, "Camera Name", "This is the camera name.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg, "Vertical field of view", "Camera's vertical (y axis) field of view in degrees.")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_width, "Image width", "Image width")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_height, "Image height", "Image height");
            }
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for camera sensor");
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kImageMessageType];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);

        m_imagePublisher = ros2Node->create_publisher<sensor_msgs::msg::Image>(fullTopic.data(), publisherConfig.GetQoS());

        m_cameraSensor.emplace(CameraSensorDescription{m_cameraName, m_VerticalFieldOfViewDeg, m_width, m_height});
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensor.reset();
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();

        if (!m_cameraSensor) {
            return;
        }
        m_cameraSensor->RequestFrame(transform, [this](const AZ::RPI::AttachmentReadback::ReadbackResult& result) {
            const AZ::RHI::ImageDescriptor& descriptor = result.m_imageDescriptor;

            sensor_msgs::msg::Image message;
            message.encoding = sensor_msgs::image_encodings::RGBA8;

            message.width = descriptor.m_size.m_width;
            message.height = descriptor.m_size.m_height;
            message.data = std::vector<uint8_t>(result.m_dataBuffer->data(), result.m_dataBuffer->data() + result.m_dataBuffer->size());
            message.header.frame_id = GetEntity()->FindComponent<ROS2FrameComponent>()->GetFrameID().c_str();

            m_imagePublisher->publish(message);
        }) ;
    }
} // namespace ROS2
