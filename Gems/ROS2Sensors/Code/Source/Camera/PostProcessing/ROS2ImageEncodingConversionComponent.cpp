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

namespace AZStd
{
    template<>
    struct hash<ROS2::EncodingConversion>
    {
        size_t operator()(const ROS2::EncodingConversion& data) const
        {
            return (static_cast<AZ::u16>(data.encodingIn) << 8) | static_cast<AZ::u16>(data.encodingOut);
        }
    };
} // namespace AZStd

namespace ROS2
{
    namespace
    {
        const AZStd::unordered_map<ImageEncoding, const char*> ImageEncodingNames = {
            { ImageEncoding::RGBA8, "rgba8" },
            { ImageEncoding::RGB8, "rgb8" },
            { ImageEncoding::Mono8, "mono8" },
            { ImageEncoding::Mono16, "mono16" },
        };
        const AZStd::unordered_map<AZStd::string, ImageEncoding> ImageEncodingFromName = {
            { "rgba8", ImageEncoding::RGBA8 },
            { "rgb8", ImageEncoding::RGB8 },
            { "mono8", ImageEncoding::Mono8 },
            { "mono16", ImageEncoding::Mono16 },
        };

        void Rgba8ToRgb8(sensor_msgs::msg::Image& image)
        {
            const std::string inputEncoding = ImageEncodingNames.at(ImageEncoding::RGBA8);
            const std::string outputEncoding = ImageEncodingNames.at(ImageEncoding::RGB8);
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

        const AZStd::unordered_map<EncodingConversion, const AZStd::function<void(sensor_msgs::msg::Image&)>> supportedFormatChange = {
            { { ImageEncoding::RGBA8, ImageEncoding::RGB8 }, Rgba8ToRgb8 },
        };

        AZ::Outcome<void, AZStd::string> ValidateEncodingConversion(EncodingConversion newConversion)
        {
            if (newConversion.encodingIn == newConversion.encodingOut)
            {
                return AZ::Failure(AZStd::string("Conversion to same type is forbidden"));
            }
            if (supportedFormatChange.find(newConversion) == supportedFormatChange.end())
            {
                return AZ::Failure(AZStd::string::format(
                    "Unsupported encoding change from %s to %s",
                    ImageEncodingNames.at(newConversion.encodingIn),
                    ImageEncodingNames.at(newConversion.encodingOut)));
            }
            return AZ::Success();
        }

    } // namespace

    void EncodingConversion::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EncodingConversion>()
                ->Version(0)
                ->Field("EncodingIn", &EncodingConversion::encodingIn)
                ->Field("EncodingOut", &EncodingConversion::encodingOut);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<EncodingConversion>("Encoding Conversion", "Specifies encoding conversion")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConversion::encodingIn, "Encoding In", "Encoding of the input image")
                    ->EnumAttribute(ImageEncoding::RGBA8, "rgba8")
                    ->EnumAttribute(ImageEncoding::RGB8, "rgb8")
                    ->EnumAttribute(ImageEncoding::Mono8, "mono8")
                    ->EnumAttribute(ImageEncoding::Mono16, "mono16")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &EncodingConversion::ValidateInputEncoding)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConversion::encodingOut, "Encoding Out", "Encoding of the output image")
                    ->EnumAttribute(ImageEncoding::RGBA8, "rgba8")
                    ->EnumAttribute(ImageEncoding::RGB8, "rgb8")
                    ->EnumAttribute(ImageEncoding::Mono8, "mono8")
                    ->EnumAttribute(ImageEncoding::Mono16, "mono16")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &EncodingConversion::ValidateOutputEncoding);
            }
        }
    }

    AZ::Outcome<void, AZStd::string> EncodingConversion::ValidateInputEncoding(void* newValue, const AZ::Uuid& valueType)
    {
        ImageEncoding* newEncoding = static_cast<ImageEncoding*>(newValue);
        return ValidateEncodingConversion({ *newEncoding, encodingOut });
    }

    AZ::Outcome<void, AZStd::string> EncodingConversion::ValidateOutputEncoding(void* newValue, const AZ::Uuid& valueType)
    {
        ImageEncoding* newEncoding = static_cast<ImageEncoding*>(newValue);
        return ValidateEncodingConversion({ encodingIn, *newEncoding });
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

    void ROS2ImageEncodingConversionComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        const auto nameIter = ImageEncodingFromName.find(image.encoding.c_str());
        if (nameIter == ImageEncodingFromName.end())
        {
            return;
        }
        const ImageEncoding& encoding = nameIter->second;
        if (encoding != m_encodingConvertData.encodingIn)
        {
            return;
        }

        const auto convertIter = supportedFormatChange.find(m_encodingConvertData);
        if (convertIter == supportedFormatChange.end())
        {
            return;
        }

        convertIter->second(image);
    }

    AZ::u8 ROS2ImageEncodingConversionComponent::GetPriority() const
    {
        return m_priority;
    }

} // namespace ROS2
