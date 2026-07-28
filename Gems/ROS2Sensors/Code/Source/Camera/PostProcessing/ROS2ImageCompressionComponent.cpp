/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImageCompressionComponent.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2NamesBus.h>
#include <ROS2/Sensor/SensorConfigurationRequestBus.h>
#include <ROS2Sensors/Camera/CameraSensorConfiguration.h>

#include <cmath>
#include <cstring>
#include <utility>

namespace ROS2Sensors
{
    namespace
    {
        //! Suffix image_transport looks for next to the raw image topic.
        constexpr const char* CompressedTopicSuffix = "/compressed";


        //! Quantize a 32FC1 depth image to a mono16 one, which PNG can store losslessly.
        //! Samples that are not finite (no return from the depth pass) and negative distances become 0, the
        //! ROS 2 convention for "no measurement". Distances beyond the mono16 range saturate.
        sensor_msgs::msg::Image QuantizeDepthToMono16(const sensor_msgs::msg::Image& image, float depthScale)
        {
            sensor_msgs::msg::Image out;
            out.header = image.header;
            out.width = image.width;
            out.height = image.height;
            out.encoding = "mono16";
            out.step = image.width * sizeof(uint16_t);
            out.data.resize(static_cast<size_t>(out.step) * image.height);

            for (uint32_t row = 0; row < image.height; ++row)
            {
                const uint8_t* sourceRow = image.data.data() + static_cast<size_t>(row) * image.step;
                uint8_t* targetRow = out.data.data() + static_cast<size_t>(row) * out.step;
                for (uint32_t column = 0; column < image.width; ++column)
                {
                    float depth = 0.0f;
                    std::memcpy(&depth, sourceRow + static_cast<size_t>(column) * sizeof(float), sizeof(float));
                    const float scaled = std::isfinite(depth) ? depth * depthScale : 0.0f;
                    const auto sample = static_cast<uint16_t>(AZ::GetClamp(scaled, 0.0f, 65535.0f));
                    std::memcpy(targetRow + static_cast<size_t>(column) * sizeof(uint16_t), &sample, sizeof(sample));
                }
            }
            return out;
        }

        //! Raw image topic the camera on this entity publishes under the given key, via its configuration bus.
        AZ::Outcome<ROS2::TopicConfiguration> FindImageTopic(const AZ::Entity* entity, const char* imageConfigKey)
        {
            for (const AZ::Component* component : entity->GetComponents())
            {
                const AZ::EntityComponentIdPair busId(entity->GetId(), component->GetId());
                if (!ROS2::SensorConfigurationRequestBus::FindFirstHandler(busId))
                {
                    continue;
                }

                ROS2::SensorConfiguration configuration;
                ROS2::SensorConfigurationRequestBus::EventResult(
                    configuration, busId, &ROS2::SensorConfigurationRequest::GetSensorConfiguration);

                const auto topicIter = configuration.m_publishersConfigurations.find(imageConfigKey);
                if (topicIter != configuration.m_publishersConfigurations.end() &&
                    topicIter->second.m_type == CameraConstants::ImageMessageType)
                {
                    return AZ::Success(topicIter->second);
                }
            }
            return AZ::Failure();
        }
    } // namespace

    void ROS2ImageCompressionComponent::Reflect(AZ::ReflectContext* context)
    {
        ImageCompression::Settings::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImageCompressionComponent, AZ::Component>()
                ->Version(0)
                ->Field("ColorSettings", &ROS2ImageCompressionComponent::m_colorSettings)
                ->Field("DepthSettings", &ROS2ImageCompressionComponent::m_depthSettings)
                ->Field("DepthScale", &ROS2ImageCompressionComponent::m_depthScale);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2ImageCompressionComponent>(
                      "Image Compression Component", "Publishes a compressed copy of every camera image on <topic>/compressed")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageCompressionComponent::m_colorSettings,
                        "Color",
                        "Compression of the color channel, including the mono encodings it can be converted to")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageCompressionComponent::m_depthSettings,
                        "Depth",
                        "Compression of the depth channel.")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageCompressionComponent::m_depthScale,
                        "Depth Scale",
                        "Multiplier applied to depth samples before quantizing them to 16 bits. 1000 turns metres into "
                        "millimetres. Consumers have to know this value; it is not part of the message.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f);
            }
        }
    }

    void ROS2ImageCompressionComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2CameraSensor"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2ImageCompressionComponent::Activate()
    {
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2ImageCompressionComponent::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();

        AZStd::lock_guard lock(m_publishersMutex);
        m_colorPublisher.reset();
        m_depthPublisher.reset();
    }

    ROS2ImageCompressionComponent::CompressedPublisherPtr ROS2ImageCompressionComponent::CreatePublisher(const char* imageConfigKey) const
    {
        const auto topic = FindImageTopic(GetEntity(), imageConfigKey);
        if (!topic.IsSuccess())
        {
            return nullptr;
        }

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
            topic.GetValue().m_topic + CompressedTopicSuffix);

        AZ_Printf("ROS2ImageCompressionComponent", "Publishing compressed %s on %s\n", imageConfigKey, fullTopic.c_str());
        return ROS2::ROS2Interface::Get()->GetNode()->create_publisher<sensor_msgs::msg::CompressedImage>(
            fullTopic.data(), topic.GetValue().GetQoS());
    }

    ROS2ImageCompressionComponent::CompressedPublisherPtr ROS2ImageCompressionComponent::GetPublisher(CameraUtils::ImageEncoding encoding)
    {
        const bool isDepth = CameraUtils::IsDepthEncoding(encoding);
        const bool isColor = CameraUtils::IsColorEncoding(encoding);

        AZ_Assert(isDepth || isColor, "Unknown encoding, %u", encoding);

        // A disabled channel must not advertise a topic it will never publish on.
        if (isDepth && m_depthSettings.m_codec == ImageCompression::Codec::DoNotPublish)
        {
            return nullptr;
        }
        if (isColor && m_colorSettings.m_codec == ImageCompression::Codec::DoNotPublish)
        {
            return nullptr;
        }

        AZStd::lock_guard lock(m_publishersMutex);
        if (isDepth)
        {
            if (!m_depthPublisher)
            {
                m_depthPublisher = CreatePublisher(CameraConstants::DepthImageConfig);
            }
            return m_depthPublisher;
        }

        if (!m_colorPublisher)
        {
            m_colorPublisher = CreatePublisher(CameraConstants::ColorImageConfig);
        }
        return m_colorPublisher;
    }

    void ROS2ImageCompressionComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        const auto nameIter = CameraUtils::ImageEncodingFromName.find(image.encoding.c_str());
        AZ_Assert(nameIter != CameraUtils::ImageEncodingFromName.end(), "Unknown  encoding : %s", image.encoding.c_str());
        if (nameIter == CameraUtils::ImageEncodingFromName.end())
        {
            return;
        }
        const auto encoding = nameIter->second;

        auto publisher = GetPublisher(encoding);
        if (!publisher)
        {
            return;
        }
        if (CameraUtils::IsDepthEncoding(encoding))
        {
            auto quatizedImage = QuantizeDepthToMono16(image, m_depthScale);
            PublishCompressed(quatizedImage, m_depthSettings, *publisher);
            return;
        }
        if (CameraUtils::IsColorEncoding(encoding))
        {
            PublishCompressed(image, m_colorSettings, *publisher);
        }

    }

    void ROS2ImageCompressionComponent::PublishCompressed(
        const sensor_msgs::msg::Image& image,
        const ImageCompression::Settings& settings,
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>& publisher) const
    {
        if (settings.m_codec == ImageCompression::Codec::DoNotPublish)
        {
            return;
        }
        auto compressed = ImageCompression::Compress(image, settings);
        if (!compressed.IsSuccess())
        {
            AZ_ErrorOnce("ROS2ImageCompressionComponent", false, "Compression failed: %s", compressed.GetError().c_str());
            return;
        }

        sensor_msgs::msg::CompressedImage message;
        message.header = image.header;
        message.format = compressed.GetValue().m_format.c_str();
        message.data = std::move(compressed.GetValue().m_data);
        publisher.publish(std::move(message));
    }

    AZ::u8 ROS2ImageCompressionComponent::GetPriority() const
    {
        // Compress the image as it will actually be published, so run after every other handler.
        return CameraPostProcessingRequests::MIN_PRIORITY;
    }
} // namespace ROS2Sensors