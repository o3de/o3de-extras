/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2CameraSensorComponent.h"
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <sensor_msgs/distortion_models.hpp>

namespace ROS2
{
    namespace Internal
    {
        const char* kImageMessageType = "sensor_msgs::msg::Image";
        const char* kDepthImageConfig = "Depth Image";
        const char* kColorImageConfig = "Color Image";
        const char* kInfoConfig = "Camera Info";
        const char* kCameraInfoMessageType = "sensor_msgs::msg::CameraInfo";

        AZStd::pair<AZStd::string, TopicConfiguration> MakeTopicConfigurationPair(
            const AZStd::string& topic, const AZStd::string& messageType, const AZStd::string& configName)
        {
            TopicConfiguration config;
            config.m_topic = topic;
            config.m_type = messageType;
            return AZStd::make_pair(configName, config);
        }

        AZStd::string GetCameraNameFromFrame(const AZ::Entity* entity)
        {
            const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
            AZStd::string cameraName = component->GetFrameID();
            AZStd::replace(cameraName.begin(), cameraName.end(), '/', '_');
            return cameraName;
        }

    } // namespace Internal

    ROS2CameraSensorComponent::ROS2CameraSensorComponent()
    {
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakeTopicConfigurationPair("camera_image_color", Internal::kImageMessageType, Internal::kColorImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakeTopicConfigurationPair("camera_image_depth", Internal::kImageMessageType, Internal::kDepthImageConfig));
        m_sensorConfiguration.m_publishersConfigurations.insert(
            Internal::MakeTopicConfigurationPair("camera_info", Internal::kCameraInfoMessageType, Internal::kInfoConfig));
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()
                ->Version(3)
                ->Field("VerticalFieldOfViewDeg", &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg)
                ->Field("Width", &ROS2CameraSensorComponent::m_width)
                ->Field("Height", &ROS2CameraSensorComponent::m_height)
                ->Field("Depth", &ROS2CameraSensorComponent::m_depthCamera)
                ->Field("Color", &ROS2CameraSensorComponent::m_colorCamera);

            AZ::EditContext* ec = serialize->GetEditContext();
            if (ec)
            {
                ec->Class<ROS2CameraSensorComponent>("ROS2 Camera Sensor", "[Camera component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2CameraSensorComponent::m_VerticalFieldOfViewDeg,
                        "Vertical field of view",
                        "Camera's vertical (y axis) field of view in degrees.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_width, "Image width", "Image width")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_height, "Image height", "Image height")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_colorCamera, "Color Camera", "Color Camera")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2CameraSensorComponent::m_depthCamera, "Depth Camera", "Depth Camera");
            }
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();

        const auto cameraInfoPublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kInfoConfig];
        AZStd::string cameraInfoFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraInfoPublisherConfig.m_topic);
        AZ_TracePrintf("ROS2", "Creating publisher for camera info on topic %s\n", cameraInfoFullTopic.data());

        m_cameraInfoPublisher =
            ros2Node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoFullTopic.data(), cameraInfoPublisherConfig.GetQoS());

        const CameraSensorDescription description{
            Internal::GetCameraNameFromFrame(GetEntity()), m_VerticalFieldOfViewDeg, m_width, m_height
        };
        if (m_colorCamera)
        {
            const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kColorImageConfig];
            AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
            auto publisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());
            m_cameraSensorsWithPublihsers.emplace_back(CreatePair<CameraColorSensor>(publisher, description));
        }
        if (m_depthCamera)
        {
            const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kDepthImageConfig];
            AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
            auto publisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());
            m_cameraSensorsWithPublihsers.emplace_back(CreatePair<CameraDepthSensor>(publisher, description));
        }
        const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        AZ_Assert(component, "Entity has no ROS2FrameComponent");
        m_frameName = component->GetFrameID();
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensorsWithPublihsers.clear();
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        const AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();
        const auto timestamp = ROS2Interface::Get()->GetROSTimestamp();
        std_msgs::msg::Header ros_header;
        if (!m_cameraSensorsWithPublihsers.empty())
        {
            const auto& camera_descritpion = m_cameraSensorsWithPublihsers.front().second->GetCameraSensorDescription();
            const auto& cameraIntrinsics = camera_descritpion.m_cameraIntrinsics;
            sensor_msgs::msg::CameraInfo cameraInfo;
            ros_header.stamp = timestamp;
            ros_header.frame_id = m_frameName.c_str();
            cameraInfo.header = ros_header;
            cameraInfo.width = m_width;
            cameraInfo.height = m_height;
            cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            AZ_Assert(cameraIntrinsics.size() == 9, "camera matrix should have 9 elements");
            AZ_Assert(cameraInfo.k.size() == 9, "camera matrix should have 9 elements");
            AZStd::copy(cameraIntrinsics.begin(), cameraIntrinsics.end(), cameraInfo.k.begin());
            cameraInfo.p = { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2], 0, cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5], 0,
                             cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8], 0 };
            m_cameraInfoPublisher->publish(cameraInfo);
        }
        for (auto& [publisher, sensor] : m_cameraSensorsWithPublihsers)
        {
            sensor->RequestMessagePublication(publisher, transform, ros_header);
        }
    }
} // namespace ROS2
