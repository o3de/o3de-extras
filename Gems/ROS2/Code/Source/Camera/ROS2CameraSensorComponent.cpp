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

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <sensor_msgs/distortion_models.hpp>

namespace ROS2
{
    ROS2CameraSensorComponent::ROS2CameraSensorComponent(
        const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration)
        : m_cameraConfiguration(cameraConfiguration)
    {
        m_sensorConfiguration = sensorConfiguration;
    }

    void ROS2CameraSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        auto* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<ROS2CameraSensorComponent, ROS2SensorComponent>()->Version(4)->Field(
                "CameraSensorConfig", &ROS2CameraSensorComponent::m_cameraConfiguration);
        }
    }

    void ROS2CameraSensorComponent::Activate()
    {
        ROS2SensorComponent::Activate();

        const auto cameraInfoPublisherConfig = m_sensorConfiguration.m_publishersConfigurations[CameraConstants::InfoConfig];
        AZStd::string cameraInfoFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraInfoPublisherConfig.m_topic);
        AZ_TracePrintf("ROS2", "Creating publisher for camera info on topic %s\n", cameraInfoFullTopic.data());

        auto ros2Node = ROS2Interface::Get()->GetNode();
        m_cameraInfoPublisher =
            ros2Node->create_publisher<sensor_msgs::msg::CameraInfo>(cameraInfoFullTopic.data(), cameraInfoPublisherConfig.GetQoS());

<<<<<<< HEAD
        if (m_cameraConfiguration.m_colorCamera && m_cameraConfiguration.m_depthCamera)
        {
            AddImageSource<CameraRGBDSensor>();
=======
        const CameraSensorDescription description{
            GetCameraNameFromFrame(GetEntity()), m_verticalFieldOfViewDeg, m_width, m_height, GetEntityId()
        };
        if (m_colorCamera)
        {
            const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[CameraConstants::ColorImageConfig];
            AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
            auto publisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());
            m_imagePublishers.emplace_back(publisher);
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
        }
        else if (m_cameraConfiguration.m_colorCamera)
        {
<<<<<<< HEAD
            AddImageSource<CameraColorSensor>();
        }
        else if (m_cameraConfiguration.m_depthCamera)
        {
            AddImageSource<CameraDepthSensor>();
=======
            const auto cameraImagePublisherConfig = m_sensorConfiguration.m_publishersConfigurations[CameraConstants::DepthImageConfig];
            AZStd::string cameraImageFullTopic = ROS2Names::GetNamespacedName(GetNamespace(), cameraImagePublisherConfig.m_topic);
            auto publisher =
                ros2Node->create_publisher<sensor_msgs::msg::Image>(cameraImageFullTopic.data(), cameraImagePublisherConfig.GetQoS());
            m_imagePublishers.emplace_back(publisher);
        }

        if (m_colorCamera && m_depthCamera)
        {
            m_cameraSensor = AZStd::make_shared<CameraRGBDSensor>(description);
        }
        else if (m_colorCamera)
        {
            m_cameraSensor = AZStd::make_shared<CameraColorSensor>(description);
        }
        else if (m_depthCamera)
        {
            m_cameraSensor = AZStd::make_shared<CameraDepthSensor>(description);
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
        }

        const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        AZ_Assert(component, "Entity has no ROS2FrameComponent");
        m_frameName = component->GetFrameID();
    }

    void ROS2CameraSensorComponent::Deactivate()
    {
        m_cameraSensor.reset();
        m_imagePublishers.clear();
        ROS2SensorComponent::Deactivate();
    }

    void ROS2CameraSensorComponent::FrequencyTick()
    {
        const AZ::Transform transform = GetEntity()->GetTransform()->GetWorldTM();
        const auto timestamp = ROS2Interface::Get()->GetROSTimestamp();
        std_msgs::msg::Header ros_header;
        if (!m_imagePublishers.empty() && m_cameraSensor)
        {
            const auto& cameraDescription = m_cameraSensor->GetCameraSensorDescription();
            const auto& cameraIntrinsics = cameraDescription.m_cameraIntrinsics;
            sensor_msgs::msg::CameraInfo cameraInfo;
            ros_header.stamp = timestamp;
            ros_header.frame_id = m_frameName.c_str();
            cameraInfo.header = ros_header;
            cameraInfo.width = m_cameraConfiguration.m_width;
            cameraInfo.height = m_cameraConfiguration.m_height;
            cameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
            AZ_Assert(cameraIntrinsics.size() == 9, "camera matrix should have 9 elements");
            AZ_Assert(cameraInfo.k.size() == 9, "camera matrix should have 9 elements");
            AZStd::copy(cameraIntrinsics.begin(), cameraIntrinsics.end(), cameraInfo.k.begin());
            cameraInfo.p = { cameraInfo.k[0], cameraInfo.k[1], cameraInfo.k[2], 0, cameraInfo.k[3], cameraInfo.k[4], cameraInfo.k[5], 0,
                             cameraInfo.k[6], cameraInfo.k[7], cameraInfo.k[8], 0 };
            m_cameraInfoPublisher->publish(cameraInfo);
            m_cameraSensor->RequestMessagePublication(m_imagePublishers, transform, ros_header );
        }
    }

    AZStd::string ROS2CameraSensorComponent::GetCameraNameFromFrame(const AZ::Entity* entity) const
    {
        const auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(entity);
        AZ_Assert(component, "Entity %s has no ROS2CameraSensorComponent", entity->GetName().c_str());
        if (component)
        {
            AZStd::string cameraName = component->GetFrameID();
            AZStd::replace(cameraName.begin(), cameraName.end(), '/', '_');
            return cameraName;
        }
        return AZStd::string{};
    }
} // namespace ROS2

