/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ImageCompression.h"
#include <Camera/CameraUtilities.h>

#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Configuration of a compressed image publisher, shared by the editor and the game component.
    struct ImageCompressionConfiguration
    {
        AZ_TYPE_INFO(ImageCompressionConfiguration, ROS2Sensors::ImageCompressionConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        //! Camera channel the compressed stream belongs to. It selects the camera image topic the result is
        //! published next to; there is no separate topic to configure.
        enum class Channel : AZ::u8
        {
            Color = 0,
            Depth = 1,
        };

        //! Channel depth/color, deduced from its encoding.
        static Channel EncodingToChannel(CameraUtils::ImageEncoding encoding);

        //! Appended to the camera image topic of the selected channel, following the image_transport convention.
        static constexpr char TopicSuffix[] = "/compressed";

        Channel m_channel = Channel::Color; //!< Camera channel to mirror.

        ImageCompression::Settings m_settings; //!< Codec and codec parameters.

    private:
        //! Text of the "Topic" label: the topic the compressed image ends up on.
        AZStd::string GetTopicUiComment() const;
    };
} // namespace ROS2Sensors
