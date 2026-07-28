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
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/base.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace ROS2Sensors::ImageCompression
{
    //! Codec used to compress an image.
    enum class Codec : AZ::u8
    {
        Jpeg = 0, //!< Lossy, 8-bit only. Best ratio for color images.
        Png = 1, //!< Lossless. Required for mono16 and quantized depth.
        DoNotPublish = 255 //!< Deactivate compression
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

    //! A compressed image payload together with the value to put in CompressedImage::format.
    struct CompressionResult
    {
        std::vector<uint8_t> m_data;
        AZStd::string m_format;
    };

    //! Whether this build has JPEG support compiled in.
    //! JPEG relies on an optional libjpeg dependency; PNG is always available.
    bool IsCodecAvailable(Codec codec);

    //! Compress a raw image message.
    //! @param image source image; `encoding`, `width`, `height`, `step` and `data` must be consistent.
    //! @param settings codec and codec parameters.
    //! @return the compressed payload and format string, or a message describing why the image
    //!         cannot be compressed with the requested codec.
    //! @note mono16 requires PNG. Alpha is dropped when compressing rgba8 with JPEG.
    //!       Float encodings are not supported; quantize depth to mono16 before calling.
    AZ::Outcome<CompressionResult, AZStd::string> Compress(const sensor_msgs::msg::Image& image, const Settings& settings);
} // namespace ROS2Sensors::ImageCompression
