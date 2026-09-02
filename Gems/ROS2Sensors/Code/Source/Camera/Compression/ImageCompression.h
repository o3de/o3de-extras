/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/Crc.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/base.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ROS2Sensors::ImageCompression
{
    //! Codec used to compress an image.
    enum class Codec : AZ::u8
    {
        Jpeg = 0, //!< Lossy. Best ratio for color images. Falls back to PNG for encodings wider than 8 bits.
        Png = 1, //!< Lossless. The only option for mono16 and quantized depth.
    };

    //! Parameters controlling the compression.
    struct Settings
    {
        AZ_TYPE_INFO(Settings, ROS2Sensors::ImageCompressionSettingsTypeId);
        static void Reflect(AZ::ReflectContext* context);

        static constexpr int MinJpegQuality = 1;
        static constexpr int MaxJpegQuality = 100;
        static constexpr int MinPngCompressionLevel = 0;
        static constexpr int MaxPngCompressionLevel = 9;

        Codec m_codec = Codec::Png;

        //! JPEG quality, 1 (smallest) to 100 (best). Ignored for PNG.
        int m_jpegQuality = 80;

        //! zlib compression level, 0 (fastest) to 9 (smallest). Ignored for JPEG.
        int m_pngCompressionLevel = 6;

    private:
        AZ::Crc32 GetJpegVisibility() const;
        AZ::Crc32 GetPngVisibility() const;
    };

    //! Whether this build has JPEG support compiled in.
    //! JPEG relies on an optional libjpeg dependency; PNG is always available.
    bool IsCodecAvailable(Codec codec);

    //! Compress a raw image message.
    //! @param image source image; `encoding`, `width`, `height`, `step` and `data` must be consistent.
    //! @param settings codec and codec parameters.
    //! @return a message ready to publish, carrying the source header, or a description of why the image
    //!         cannot be compressed with the requested codec.
    //! @note Requesting JPEG for an encoding wider than 8 bits, such as mono16, fails: baseline JPEG stores
    //!       8 bits per sample. Use PNG for those. Alpha is dropped when compressing rgba8 with JPEG.
    //!       Float encodings are not supported; quantize depth to mono16 before calling.
    AZ::Outcome<sensor_msgs::msg::CompressedImage, AZStd::string> Compress(const sensor_msgs::msg::Image& image, const Settings& settings);
} // namespace ROS2Sensors::ImageCompression
