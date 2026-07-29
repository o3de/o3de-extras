/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include "../Compression/ImageCompressionConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2Sensors/Camera/CameraPostProcessingRequestBus.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ROS2Sensors
{
    //! Compresses camera frames and publishes them as sensor_msgs::msg::CompressedImage.
    //! Game counterpart of ROS2ImageCompressionEditorComponent, which builds it; it is not added by hand,
    //! so it creates its publisher only in a running simulation and never in the Editor.
    class ROS2ImageCompressionComponent
        : public AZ::Component
        , public CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2ImageCompressionComponent, ROS2Sensors::ROS2ImageCompressionComponentTypeId, AZ::Component);
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        ROS2ImageCompressionComponent() = default;
        explicit ROS2ImageCompressionComponent(
            ImageCompressionConfiguration::Channel channel,
            const ROS2::TopicConfiguration& sourceCameraTopicConfig,
            const ImageCompression::Settings& settings);
        ~ROS2ImageCompressionComponent() override = default;

        void Activate() override;
        void Deactivate() override;

        //! CameraPostProcessingRequestBus::Handler overrides
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override;

    private:
        void CompressAndPublish(const sensor_msgs::msg::Image& image);
        //! Created in Activate before the bus is connected and reset after it is disconnected, so the frame
        //! path only ever reads it. The post-processing bus dispatches frames concurrently.
        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> m_publisher;
        ImageCompressionConfiguration::Channel m_channel;
        ROS2::TopicConfiguration m_sourceCameraTopicConfig;
        ImageCompression::Settings m_compressionSettings;
    };
} // namespace ROS2Sensors
