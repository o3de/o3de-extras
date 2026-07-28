/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ImageCompression.h"
#include "Camera/CameraUtilities.h"
#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/Camera/CameraPostProcessingRequestBus.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ROS2Sensors
{
    //! Publishes a compressed copy of every image the camera on this entity produces.
    //!
    //! Image topics are discovered from the camera's publisher configuration; each gets a
    //! "<raw topic>/compressed" counterpart with the same QoS, as image_transport expects. Raw images are
    //! not modified. Depth is quantized to mono16 first, so it always uses PNG, as does mono16 itself.
    class ROS2ImageCompressionComponent
        : public AZ::Component
        , public CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2ImageCompressionComponent, ROS2Sensors::ROS2ImageCompressionComponentTypeId, AZ::Component);
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        ROS2ImageCompressionComponent() = default;
        ~ROS2ImageCompressionComponent() override = default;

        void Activate() override;
        void Deactivate() override;

        //! CameraPostProcessingRequestBus::Handler overrides
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override;

    private:

        using CompressedPublisherPtr = std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>>;

        //! Publisher for the channel this encoding comes from, created on first use.
        //! @note Not created in Activate(), where the camera may not be active and has no topics to report.
        CompressedPublisherPtr GetPublisher(CameraUtils::ImageEncoding encoding);

        //! Publisher mirroring one of the camera's image topics, e.g. CameraConstants::ColorImageConfig.
        //! @return null if the camera does not publish that topic.
        CompressedPublisherPtr CreatePublisher(const char* imageConfigKey) const;

        //! Compress an image and publish the result. Errors are reported, not propagated.
        void PublishCompressed(
            const sensor_msgs::msg::Image& image, const ImageCompression::Settings& settings, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>& publisher) const;

        ImageCompression::Settings m_colorSettings;
        //! Depth cannot use JPEG, so PNG is both the default and what ImageCompression falls back to.
        ImageCompression::Settings m_depthSettings = { ImageCompression::Codec::Png };
        float m_depthScale = 1000.0f; //! Depth scale before compression

        //! Guarded by the mutex; the post-processing bus dispatches frames concurrently.
        CompressedPublisherPtr m_colorPublisher;
        CompressedPublisherPtr m_depthPublisher;
        AZStd::mutex m_publishersMutex;
    };
} // namespace ROS2Sensors