/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ImageCompressionConfiguration.h"

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2Sensors
{
    ImageCompressionConfiguration::Channel ImageCompressionConfiguration::EncodingToChannel(CameraUtils::ImageEncoding encoding)
    {
        if (encoding == CameraUtils::ImageEncoding::RGBA8 || encoding == CameraUtils::ImageEncoding::RGB8)
        {
            return ImageCompressionConfiguration::Channel::Color;
        }
        if (encoding == CameraUtils::ImageEncoding::Mono8 || encoding == CameraUtils::ImageEncoding::Mono16 ||
            encoding == CameraUtils::ImageEncoding::Depth32FC1)
        {
            return ImageCompressionConfiguration::Channel::Depth;
        }

        AZ_Assert(false, "Unknown or unsupported encoding %u ", encoding);
        return ImageCompressionConfiguration::Channel::Color;
    }

    void ImageCompressionConfiguration::Reflect(AZ::ReflectContext* context)
    {
        ImageCompression::Settings::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ImageCompressionConfiguration>()
                ->Version(0)
                ->Field("channel", &ImageCompressionConfiguration::m_channel)
                ->Field("settings", &ImageCompressionConfiguration::m_settings);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ImageCompressionConfiguration>("Image Compression Configuration", "Configuration of a compressed image publisher")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &ImageCompressionConfiguration::m_channel,
                        "Channel",
                        "Camera channel to mirror. The compressed image is published next to the image topic of that "
                        "channel, with the QoS the camera uses for it.")
                    ->EnumAttribute(Channel::Color, "color")
                    ->EnumAttribute(Channel::Depth, "depth")
                    // The label below spells out the resulting topic, so re-evaluate it on every edit.
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::AttributesAndValues)

                    ->UIElement(AZ::Edit::UIHandlers::Label, "Topic", "")
                    ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "Topic")
                    ->Attribute(AZ::Edit::Attributes::ValueText, &ImageCompressionConfiguration::GetTopicUiComment)

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ImageCompressionConfiguration::m_settings,
                        "Compression Settings",
                        "Compression settings")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    AZStd::string ImageCompressionConfiguration::GetTopicUiComment() const
    {
        return AZStd::string::format(
            "The camera's %s image topic with <b>%s</b> appended.", m_channel == Channel::Depth ? "depth" : "color", TopicSuffix);
    }
} // namespace ROS2Sensors
