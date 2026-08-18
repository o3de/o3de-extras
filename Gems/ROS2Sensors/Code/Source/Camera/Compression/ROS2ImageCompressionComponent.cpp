/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImageCompressionComponent.h"
#include "Camera/PostProcessing/ROS2ImageEncodingConversionComponent.h"
#include "Camera/ROS2CameraSensorComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2Sensors/Camera/CameraSensorConfiguration.h>

namespace ROS2Sensors
{
    namespace
    {
        //! Scale used by the fallback quantization of 32FC1 depth, turning metres into millimetres.
        constexpr float DepthFallbackScaleFactor = 1000.0f;
    } // namespace

    ROS2ImageCompressionComponent::ROS2ImageCompressionComponent(
        ImageCompressionConfiguration::Channel channel,
        const ROS2::TopicConfiguration& sourceCameraTopicConfig,
        const ImageCompression::Settings& settings)
        : m_channel(channel)
        , m_sourceCameraTopicConfig(sourceCameraTopicConfig)
        , m_compressionSettings(settings)
    {
    }

    void ROS2ImageCompressionComponent::Reflect(AZ::ReflectContext* context)
    {
        ImageCompressionConfiguration::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImageCompressionComponent, AZ::Component>()
                ->Version(1)
                ->Field("sourceConfig", &ROS2ImageCompressionComponent::m_sourceCameraTopicConfig)
                ->Field("settings", &ROS2ImageCompressionComponent::m_compressionSettings)
                ->Field("channel", &ROS2ImageCompressionComponent::m_channel);
        }
    }

    void ROS2ImageCompressionComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2CameraSensor"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2ImageCompressionComponent::Activate()
    {
        AZStd::string cameraNamespace;
        if (const auto* frame = GetEntity()->FindComponent<ROS2::ROS2FrameComponent>())
        {
            cameraNamespace = frame->GetNamespace();
        }

        AZStd::string fullTopic;
        ROS2::ROS2NamesRequestBus::BroadcastResult(
            fullTopic,
            &ROS2::ROS2NamesRequests::GetNamespacedName,
            cameraNamespace,
            m_sourceCameraTopicConfig.m_topic + ImageCompressionConfiguration::TopicSuffix);

        AZ_Printf("ROS2ImageCompressionComponent", "Publishing compressed on %s\n", fullTopic.c_str());

        m_publisher = ROS2::ROS2Interface::Get()->GetNode()->create_publisher<sensor_msgs::msg::CompressedImage>(
            fullTopic.data(), m_sourceCameraTopicConfig.GetQoS());

        // Connect only once the publisher exists, so a dispatch can never observe a half-built component.
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2ImageCompressionComponent::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();
        m_publisher.reset();
    }

    void ROS2ImageCompressionComponent::CompressAndPublish(const sensor_msgs::msg::Image& image)
    {
        auto compressed = ImageCompression::Compress(image, m_compressionSettings);
        if (!compressed.IsSuccess())
        {
            AZ_Error("ROS2ImageCompressionComponent", false, "Compression failed: %s", compressed.GetError().c_str());
            return;
        }
        m_publisher->publish(compressed.TakeValue());
    }

    void ROS2ImageCompressionComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        const auto nameIter = CameraUtils::ImageEncodingFromName.find(image.encoding.c_str());
        AZ_Assert(nameIter != CameraUtils::ImageEncodingFromName.end(), "Unknown encoding : %s", image.encoding.c_str());
        if (nameIter == CameraUtils::ImageEncodingFromName.end())
        {
            return;
        }
        const auto encoding = nameIter->second;
        if (m_channel != ImageCompressionConfiguration::EncodingToChannel(encoding))
        {
            // the message was not for this component.
            return;
        }

        if (encoding == CameraUtils::ImageEncoding::Depth32FC1)
        {
            AZ_WarningOnce(
                "ROS2ImageCompressionComponent",
                false,
                "No codec accepts Depth32FC1, so it is converted to mono16 with a scale of %.0f. Add a "
                "ROS2ImageEncodingConversionComponent to the entity to pick the encoding and scale yourself.",
                DepthFallbackScaleFactor);
            sensor_msgs::msg::Image imageCpy = image; // copy image to do not modify raw image
            EncodingConversion conversion;
            conversion.encodingIn = CameraUtils::ImageEncoding::Depth32FC1;
            conversion.encodingOut = CameraUtils::ImageEncoding::Mono16;
            conversion.m_scaleFactor = DepthFallbackScaleFactor;

            if (!ApplyEncodingConversion(imageCpy, conversion))
            {
                // Without the quantization the copy is still float, which no codec accepts.
                AZ_Error("ROS2ImageCompressionComponent", false, "Could not convert Depth32FC1 to mono16.");
                return;
            }
            CompressAndPublish(imageCpy);
            return;
        }
        CompressAndPublish(image);
    }

    AZ::u8 ROS2ImageCompressionComponent::GetPriority() const
    {
        return CameraPostProcessingRequests::MIN_PRIORITY;
    }
} // namespace ROS2Sensors
