/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImageEncodingConversionComponent.h"
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>

#include <cmath>
#include <cstring>
#include <limits>

namespace AZStd
{
    template<>
    struct hash<ROS2Sensors::EncodingConversion>
    {
        size_t operator()(const ROS2Sensors::EncodingConversion& data) const
        {
            return (static_cast<AZ::u16>(data.encodingIn) << 8) | static_cast<AZ::u16>(data.encodingOut);
        }
    };
} // namespace AZStd

namespace ROS2Sensors
{
    namespace
    {
        void Rgba8ToRgb8(sensor_msgs::msg::Image& image, [[maybe_unused]] float scaleFactor)
        {
            const std::string inputEncoding = CameraUtils::ImageEncodingNames.at(CameraUtils::ImageEncoding::RGBA8);
            const std::string outputEncoding = CameraUtils::ImageEncodingNames.at(CameraUtils::ImageEncoding::RGB8);
            AZ_Assert(image.encoding == inputEncoding, "Image encoding is %s, expected %s", image.encoding.c_str(), inputEncoding.c_str());
            AZ_Assert(image.step == image.width * 4, "Image step (%d) is not width * 4 (%d)", image.step, image.width * 4);
            AZ_Assert(
                image.data.size() == image.step * image.height,
                "Image data size (%d) is not step * height (%d)",
                image.data.size(),
                image.step * image.height);

            // Perform conversion in place
            for (size_t pixelId = 0; pixelId < image.width * image.height; ++pixelId)
            {
                size_t pixelOffsetIn = pixelId * 4;
                size_t pixelOffsetOut = pixelId * 3;
                image.data[pixelOffsetOut] = image.data[pixelOffsetIn];
                image.data[pixelOffsetOut + 1] = image.data[pixelOffsetIn + 1];
                image.data[pixelOffsetOut + 2] = image.data[pixelOffsetIn + 2];
            }
            image.encoding = outputEncoding;
            image.step = image.width * 3;
            image.data.resize(image.step * image.height);
        }

        //! Quantize a 32FC1 depth image into an unsigned integer encoding, in place.
        //!
        //! @param image 32FC1 depth image, rewritten in place.
        //! @param scaleFactor multiplier applied before quantizing.
        //! @param outputEncoding encoding name to stamp on the result; has to match TargetSample.
        //! @note Samples carrying no measurement (not finite, negative) and samples the target type cannot
        //!       represent become 0, the sensor_msgs convention for "no reading". Saturating instead would turn
        //!       everything past the range into a solid wall at the maximum distance.
        template<typename TargetSample>
        void QuantizeDepth(sensor_msgs::msg::Image& image, float scaleFactor, CameraUtils::ImageEncoding outputEncoding)
        {
            const std::string inputEncoding = CameraUtils::ImageEncodingNames.at(CameraUtils::ImageEncoding::Depth32FC1);
            AZ_Assert(image.encoding == inputEncoding, "Image encoding is %s, expected %s", image.encoding.c_str(), inputEncoding.c_str());
            AZ_Assert(
                image.step >= image.width * sizeof(float),
                "Image step (%u) is smaller than width * sizeof(float) (%zu)",
                image.step,
                image.width * sizeof(float));
            AZ_Assert(
                image.data.size() >= static_cast<size_t>(image.step) * image.height,
                "Image data size (%zu) is smaller than step * height (%zu)",
                image.data.size(),
                static_cast<size_t>(image.step) * image.height);

            constexpr auto MaxSample = static_cast<float>(std::numeric_limits<TargetSample>::max());
            const size_t outputStep = static_cast<size_t>(image.width) * sizeof(TargetSample);

            // Every sample narrows from 4 bytes to 1 or 2, so each write lands at or before the pixel just
            // read. That makes a forward pass safe to run in place.
            for (uint32_t row = 0; row < image.height; ++row)
            {
                for (uint32_t column = 0; column < image.width; ++column)
                {
                    float depth = 0.0f;
                    std::memcpy(
                        &depth,
                        image.data.data() + static_cast<size_t>(row) * image.step + static_cast<size_t>(column) * sizeof(float),
                        sizeof(depth));

                    const float scaled = depth * scaleFactor;
                    const auto sample = (std::isfinite(scaled) && scaled >= 0.0f && scaled <= MaxSample)
                        ? static_cast<TargetSample>(scaled)
                        : TargetSample{ 0 };

                    std::memcpy(
                        image.data.data() + static_cast<size_t>(row) * outputStep + static_cast<size_t>(column) * sizeof(TargetSample),
                        &sample,
                        sizeof(sample));
                }
            }

            image.encoding = CameraUtils::ImageEncodingNames.at(outputEncoding);
            image.step = static_cast<uint32_t>(outputStep);
            image.data.resize(outputStep * image.height);
        }

        void Depth32FC1ToMono8(sensor_msgs::msg::Image& image, float scaleFactor)
        {
            QuantizeDepth<uint8_t>(image, scaleFactor, CameraUtils::ImageEncoding::Mono8);
        }

        void Depth32FC1ToMono16(sensor_msgs::msg::Image& image, float scaleFactor)
        {
            QuantizeDepth<uint16_t>(image, scaleFactor, CameraUtils::ImageEncoding::Mono16);
        }

        const AZStd::unordered_map<EncodingConversion, AZStd::function<void(sensor_msgs::msg::Image&, float)>> supportedFormatChange = {
            { { CameraUtils::ImageEncoding::RGBA8, CameraUtils::ImageEncoding::RGB8 }, Rgba8ToRgb8 },
            { { CameraUtils::ImageEncoding::Depth32FC1, CameraUtils::ImageEncoding::Mono8 }, Depth32FC1ToMono8 },
            { { CameraUtils::ImageEncoding::Depth32FC1, CameraUtils::ImageEncoding::Mono16 }, Depth32FC1ToMono16 },
        };

        AZ::Outcome<void, AZStd::string> ValidateEncodingConversion(const EncodingConversion& newConversion)
        {
            if (newConversion.encodingIn == newConversion.encodingOut)
            {
                return AZ::Failure(AZStd::string("Conversion to same type is forbidden"));
            }
            if (supportedFormatChange.find(newConversion) == supportedFormatChange.end())
            {
                return AZ::Failure(AZStd::string::format(
                    "<b>\U000026A0\U0000FE0F Unsupported encoding change from %s to %s</b>",
                    CameraUtils::ImageEncodingNames.at(newConversion.encodingIn),
                    CameraUtils::ImageEncodingNames.at(newConversion.encodingOut)));
            }
            return AZ::Success();
        }

    } // namespace

    bool ApplyEncodingConversion(sensor_msgs::msg::Image& image, const EncodingConversion& conversion)
    {
        const auto nameIter = CameraUtils::ImageEncodingFromName.find(image.encoding.c_str());
        if (nameIter == CameraUtils::ImageEncodingFromName.end() || nameIter->second != conversion.encodingIn)
        {
            return false;
        }

        const auto convertIter = supportedFormatChange.find(conversion);
        if (convertIter == supportedFormatChange.end())
        {
            return false;
        }

        convertIter->second(image, conversion.m_scaleFactor);
        return true;
    }

    void EncodingConversion::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EncodingConversion>()
                ->Version(0)
                ->Field("EncodingIn", &EncodingConversion::encodingIn)
                ->Field("EncodingOut", &EncodingConversion::encodingOut)
                ->Field("ScaleFactor", &EncodingConversion::m_scaleFactor);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<EncodingConversion>("Encoding Conversion", "Specifies encoding conversion")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConversion::encodingIn, "Encoding In", "Encoding of the input image")
                    ->EnumAttribute(CameraUtils::ImageEncoding::RGBA8, "rgba8")
                    ->EnumAttribute(CameraUtils::ImageEncoding::Depth32FC1, "Depth32FC1")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConversion::encodingOut, "Encoding Out", "Encoding of the output image")
                    ->EnumAttribute(CameraUtils::ImageEncoding::RGBA8, "rgba8")
                    ->EnumAttribute(CameraUtils::ImageEncoding::RGB8, "rgb8")
                    ->EnumAttribute(CameraUtils::ImageEncoding::Mono8, "mono8")
                    ->EnumAttribute(CameraUtils::ImageEncoding::Mono16, "mono16")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)

                    ->UIElement(AZ::Edit::UIHandlers::Label, "Conversion summary", "")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                    ->Attribute(AZ::Edit::Attributes::ValueText, &EncodingConversion::GetEncodingUiComment)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &EncodingConversion::m_scaleFactor,
                        "Scale factor",
                        "Multiplier applied to depth samples before they are quantized. 1000 turns metres into "
                        "millimetres. Consumers have to know this value; it is not part of the message.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &EncodingConversion::GetScaleFactorVisibility);
            }
        }
    }

    AZStd::string EncodingConversion::GetEncodingUiComment() const
    {
        const auto validation = ValidateEncodingConversion(*this);
        if (validation.IsSuccess())
        {
            return AZStd::string::format(
                "%s will be converted to %s.",
                CameraUtils::ImageEncodingNames.at(encodingIn),
                CameraUtils::ImageEncodingNames.at(encodingOut));
        }
        // ApplyPostProcessing skips a pair it has no conversion for, leaving the image as the camera made it.
        return AZStd::string::format("<b>No conversion will be applied.</b><br>%s.", validation.GetError().c_str());
    }

    AZ::Crc32 EncodingConversion::GetScaleFactorVisibility() const
    {
        return encodingIn == ImageEncoding::Depth32FC1
            ? AZ::Edit::PropertyVisibility::Show
            : AZ::Edit::PropertyVisibility::Hide;
    }


    void ROS2ImageEncodingConversionComponent::Reflect(AZ::ReflectContext* context)
    {
        EncodingConversion::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImageEncodingConversionComponent, AZ::Component>()
                ->Version(0)
                ->Field("Priority", &ROS2ImageEncodingConversionComponent::m_priority)
                ->Field("EncodingConvertData", &ROS2ImageEncodingConversionComponent::m_encodingConvertData);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2ImageEncodingConversionComponent>(
                      "Image Encoding Conversion Component", "Converts image encoding to a different encoding")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageEncodingConversionComponent::m_priority,
                        "Priority",
                        "Priority of the post processing. The higher the number the later the post processing is applied.")
                    ->Attribute(AZ::Edit::Attributes::Min, CameraPostProcessingRequests::MIN_PRIORITY)
                    ->Attribute(AZ::Edit::Attributes::Max, CameraPostProcessingRequests::MAX_PRIORITY)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageEncodingConversionComponent::m_encodingConvertData,
                        "Encoding Conversion",
                        "Specifies the encoding conversion to apply");
            }
        }
    }

    void ROS2ImageEncodingConversionComponent::Activate()
    {
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2ImageEncodingConversionComponent::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();
    }

    const EncodingConversion& ROS2ImageEncodingConversionComponent::GetEncodingConversion() const
    {
        return m_encodingConvertData;
    }

    void ROS2ImageEncodingConversionComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        ApplyEncodingConversion(image, m_encodingConvertData);
    }

    AZ::u8 ROS2ImageEncodingConversionComponent::GetPriority() const
    {
        return m_priority;
    }

} // namespace ROS2Sensors
