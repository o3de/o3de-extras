/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImageFormatConvertComponent.h"
#include "AzCore/RTTI/RTTIMacros.h"
#include "AzCore/Serialization/EditContextConstants.inl"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>

namespace AZStd
{
    template<>
    struct hash<ROS2::EncodingConvertData>
    {
        size_t operator()(const ROS2::EncodingConvertData& data) const
        {
            return (static_cast<AZ::u16>(data.encodingIn) << 8) | static_cast<AZ::u16>(data.encodingOut);
        }
    };
} // namespace AZStd

namespace ROS2
{
    namespace
    {
        void Rgba8ToRgb8(sensor_msgs::msg::Image& image)
        {
            AZ_Assert(image.encoding == "rgba8", "Image encoding is not rgba8");
            AZ_Assert(image.step == image.width * 4, "Image step is not width * 4");
            AZ_Assert(image.data.size() == image.step * image.height, "Image data size is not step * height");

            for (size_t pixelId = 0; pixelId < image.width * image.height; ++pixelId)
            {
                size_t pixelOffsetIn = pixelId * 4;
                size_t pixelOffsetOut = pixelId * 3;
                image.data[pixelOffsetOut] = image.data[pixelOffsetIn];
                image.data[pixelOffsetOut + 1] = image.data[pixelOffsetIn + 1];
                image.data[pixelOffsetOut + 2] = image.data[pixelOffsetIn + 2];
            }
            image.encoding = "rgb8";
            image.step = image.width * 3;
            image.data.resize(image.step * image.height);
        }

        const AZStd::unordered_map<ImageEncoding, const char*> ImageEncodingNames = {
            { ImageEncoding::rgba8, "rgba8" },
            { ImageEncoding::rgb8, "rgb8" },
            { ImageEncoding::mono8, "mono8" },
            { ImageEncoding::mono16, "mono16" },
        };
        const AZStd::unordered_map<AZStd::string, ImageEncoding> ImageEncodingFromName = {
            { "rgba8", ImageEncoding::rgba8 },
            { "rgb8", ImageEncoding::rgb8 },
            { "mono8", ImageEncoding::mono8 },
            { "mono16", ImageEncoding::mono16 },
        };

        const AZStd::unordered_map<EncodingConvertData, const AZStd::function<void(sensor_msgs::msg::Image&)>> supportedFormatChange = {
            { { ImageEncoding::rgba8, ImageEncoding::rgb8 }, Rgba8ToRgb8 },
        };
    } // namespace

    void EncodingConvertData::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EncodingConvertData>()
                ->Version(0)
                ->Field("EncodingIn", &EncodingConvertData::encodingIn)
                ->Field("EncodingOut", &EncodingConvertData::encodingOut);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<EncodingConvertData>("Encoding Convert Data", "Data for converting")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConvertData::encodingIn, "Encoding In", "Encoding of the input image")
                    ->EnumAttribute(ImageEncoding::rgba8, "rgba8")
                    ->EnumAttribute(ImageEncoding::rgb8, "rgb8")
                    ->EnumAttribute(ImageEncoding::mono8, "mono8")
                    ->EnumAttribute(ImageEncoding::mono16, "mono16")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &EncodingConvertData::encodingOut, "Encoding Out", "Encoding of the output image")
                    ->EnumAttribute(ImageEncoding::rgba8, "rgba8")
                    ->EnumAttribute(ImageEncoding::rgb8, "rgb8")
                    ->EnumAttribute(ImageEncoding::mono8, "mono8")
                    ->EnumAttribute(ImageEncoding::mono16, "mono16");
            }
        }
    }

    void ROS2ImageFormatConvertComponent::Reflect(AZ::ReflectContext* context)
    {
        EncodingConvertData::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImageFormatConvertComponent, AZ::Component>()
                ->Version(0)
                ->Field("Priority", &ROS2ImageFormatConvertComponent::m_priority)
                ->Field("EncodingConvertData", &ROS2ImageFormatConvertComponent::m_encodingConvertData);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2ImageFormatConvertComponent>("Image Format Convert", "Converts image format to a different format")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageFormatConvertComponent::m_priority,
                        "Priority",
                        "Priority of the post processing. The higher the number the later the post processing is applied.")
                    ->Attribute(AZ::Edit::Attributes::Min, CameraPostProcessingRequests::MIN_PRIORITY)
                    ->Attribute(AZ::Edit::Attributes::Max, CameraPostProcessingRequests::MAX_PRIORITY)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageFormatConvertComponent::m_encodingConvertData,
                        "Encoding Convert Data",
                        "Data for converting")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2ImageFormatConvertComponent::ValidateEncodingConversion);
            }
        }
    }

    void ROS2ImageFormatConvertComponent::Activate()
    {
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2ImageFormatConvertComponent::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();
    }

    void ROS2ImageFormatConvertComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        auto nameIter = ImageEncodingFromName.find(image.encoding.c_str());
        if (nameIter == ImageEncodingFromName.end())
        {
            return;
        }
        ImageEncoding encoding = nameIter->second;
        if (encoding != m_encodingConvertData.encodingIn)
        {
            return;
        }

        auto convertIter = supportedFormatChange.find(m_encodingConvertData);
        if (convertIter == supportedFormatChange.end())
        {
            return;
        }

        convertIter->second(image);
    }

    AZ::u8 ROS2ImageFormatConvertComponent::GetPriority() const
    {
        return m_priority;
    }

    AZ::Outcome<void, AZStd::string> ROS2ImageFormatConvertComponent::ValidateEncodingConversion(void* newValue, const AZ::Uuid& valueType)
    {
        EncodingConvertData* data = reinterpret_cast<EncodingConvertData*>(newValue);
        if (supportedFormatChange.find(*data) == supportedFormatChange.end())
        {
            return AZ::Failure(AZStd::string::format(
                "Unsupported encoding change from %s to %s",
                ImageEncodingNames.at(data->encodingIn),
                ImageEncodingNames.at(data->encodingOut)));
        }
        return AZ::Success();
    }

} // namespace ROS2
