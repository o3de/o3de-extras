/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ImageCompression.h"

#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_map.h>

#include <cmath>
#include <cstring>
#include <png.h>
#include <setjmp.h>

#ifdef ROS2SENSORS_WITH_JPEG
// jpeglib.h does not include the standard headers it depends on.
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include <jpeglib.h>
#endif

namespace ROS2Sensors::ImageCompression
{
    namespace
    {
        //! Layout of a supported input encoding.
        struct EncodingLayout
        {
            int m_channels = 0; //!< Samples per pixel.
            int m_bytesPerSample = 0; //!< Bytes per sample in the source data.
        };

        const AZStd::unordered_map<AZStd::string, EncodingLayout> SupportedEncodings = {
            { "rgba8", { 4, 1 } }, { "rgb8", { 3, 1 } }, { "mono8", { 1, 1 } }, { "mono16", { 1, 2 } },
        };

        //! Validate that the message fields describe a consistent buffer, and return its layout.
        AZ::Outcome<EncodingLayout, AZStd::string> GetValidatedLayout(const sensor_msgs::msg::Image& image)
        {
            const auto encodingIter = SupportedEncodings.find(image.encoding.c_str());
            if (encodingIter == SupportedEncodings.end())
            {
                return AZ::Failure(AZStd::string::format("Encoding %s cannot be compressed", image.encoding.c_str()));
            }
            const EncodingLayout& layout = encodingIter->second;

            if (image.width == 0 || image.height == 0)
            {
                return AZ::Failure(AZStd::string("Image is empty"));
            }

            const size_t minimumStep = static_cast<size_t>(image.width) * layout.m_channels * layout.m_bytesPerSample;
            if (image.step < minimumStep)
            {
                return AZ::Failure(AZStd::string::format(
                    "Image step (%zu) is smaller than a single %s row (%zu)",
                    static_cast<size_t>(image.step),
                    image.encoding.c_str(),
                    minimumStep));
            }
            const size_t minimumSize = static_cast<size_t>(image.step) * image.height;
            if (image.data.size() < minimumSize)
            {
                return AZ::Failure(AZStd::string::format(
                    "Image data size (%zu) is smaller than step * height (%zu)", image.data.size(), minimumSize));
            }

            return AZ::Success(layout);
        }

        //! Pack 16-bit samples in the big-endian order that PNG requires.
        void AppendBigEndian16(std::vector<uint8_t>& out, uint16_t value)
        {
            out.push_back(static_cast<uint8_t>(value >> 8));
            out.push_back(static_cast<uint8_t>(value & 0xFF));
        }

        //! Repack mono16 samples from host order into the big-endian order that PNG requires.
        std::vector<uint8_t> ToBigEndianMono16(const sensor_msgs::msg::Image& image)
        {
            std::vector<uint8_t> out;
            out.reserve(static_cast<size_t>(image.width) * image.height * sizeof(uint16_t));
            for (uint32_t row = 0; row < image.height; ++row)
            {
                const uint8_t* rowStart = image.data.data() + static_cast<size_t>(row) * image.step;
                for (uint32_t column = 0; column < image.width; ++column)
                {
                    uint16_t sample = 0;
                    std::memcpy(&sample, rowStart + static_cast<size_t>(column) * sizeof(uint16_t), sizeof(uint16_t));
                    AppendBigEndian16(out, sample);
                }
            }
            return out;
        }

        //! Drop the alpha channel, since baseline JPEG has no way to store it.
        std::vector<uint8_t> DropAlpha(const sensor_msgs::msg::Image& image)
        {
            std::vector<uint8_t> out;
            out.reserve(static_cast<size_t>(image.width) * image.height * 3);
            for (uint32_t row = 0; row < image.height; ++row)
            {
                const uint8_t* rowStart = image.data.data() + static_cast<size_t>(row) * image.step;
                for (uint32_t column = 0; column < image.width; ++column)
                {
                    const uint8_t* pixel = rowStart + static_cast<size_t>(column) * 4;
                    out.insert(out.end(), pixel, pixel + 3);
                }
            }
            return out;
        }

        //! libpng reports failures by longjmp-ing out of the call that failed.
        void PngWriteToVector(png_structp png, png_bytep data, png_size_t length)
        {
            auto* out = static_cast<std::vector<uint8_t>*>(png_get_io_ptr(png));
            out->insert(out->end(), data, data + length);
        }

        //! Encode a single-plane or interleaved buffer as PNG.
        //! @note Only POD locals live in this function; libpng error handling longjmps past their destructors.
        //! @param out receives the PNG stream; owned by the caller so that longjmp cannot skip its destructor.
        bool EncodePng(
            const uint8_t* pixels,
            size_t stride,
            uint32_t width,
            uint32_t height,
            int colorType,
            int bitDepth,
            int compressionLevel,
            std::vector<uint8_t>* out)
        {
            png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
            if (!png)
            {
                return false;
            }
            png_infop info = png_create_info_struct(png);
            if (!info)
            {
                png_destroy_write_struct(&png, nullptr);
                return false;
            }
            if (setjmp(png_jmpbuf(png)))
            {
                png_destroy_write_struct(&png, &info);
                return false;
            }

            png_set_write_fn(png, out, PngWriteToVector, nullptr);
            png_set_compression_level(png, compressionLevel);
            png_set_IHDR(
                png,
                info,
                width,
                height,
                bitDepth,
                colorType,
                PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_DEFAULT,
                PNG_FILTER_TYPE_DEFAULT);
            png_write_info(png, info);

            for (uint32_t row = 0; row < height; ++row)
            {
                // libpng copies each row into its own buffer before transforming it, so the source is not modified.
                png_write_row(png, const_cast<png_bytep>(pixels + static_cast<size_t>(row) * stride));
            }

            png_write_end(png, info);
            png_destroy_write_struct(&png, &info);
            return true;
        }

#ifdef ROS2SENSORS_WITH_JPEG
        struct JpegErrorManager
        {
            jpeg_error_mgr m_base;
            jmp_buf m_setjmpBuffer;
            char m_message[JMSG_LENGTH_MAX];
        };

        void JpegErrorExit(j_common_ptr cinfo)
        {
            auto* errorManager = reinterpret_cast<JpegErrorManager*>(cinfo->err);
            (*cinfo->err->format_message)(cinfo, errorManager->m_message);
            longjmp(errorManager->m_setjmpBuffer, 1);
        }

        //! Encode an 8-bit buffer as JPEG.
        //! @note Only POD locals live in this function; libjpeg error handling longjmps past their destructors.
        //! @param outBuffer on success receives a malloc'ed buffer that the caller must free().
        bool EncodeJpeg(
            const uint8_t* pixels,
            size_t stride,
            uint32_t width,
            uint32_t height,
            int channels,
            J_COLOR_SPACE colorSpace,
            int quality,
            unsigned char** outBuffer,
            unsigned long* outSize)
        {
            jpeg_compress_struct cinfo;
            JpegErrorManager errorManager;

            cinfo.err = jpeg_std_error(&errorManager.m_base);
            errorManager.m_base.error_exit = JpegErrorExit;
            if (setjmp(errorManager.m_setjmpBuffer))
            {
                AZ_Error("ImageCompression", false, "libjpeg failed: %s", errorManager.m_message);
                jpeg_destroy_compress(&cinfo);
                if (*outBuffer)
                {
                    free(*outBuffer);
                    *outBuffer = nullptr;
                }
                return false;
            }

            jpeg_create_compress(&cinfo);
            jpeg_mem_dest(&cinfo, outBuffer, outSize);
            cinfo.image_width = width;
            cinfo.image_height = height;
            cinfo.input_components = channels;
            cinfo.in_color_space = colorSpace;
            jpeg_set_defaults(&cinfo);
            jpeg_set_quality(&cinfo, quality, TRUE);
            jpeg_start_compress(&cinfo, TRUE);

            while (cinfo.next_scanline < cinfo.image_height)
            {
                // libjpeg does not modify the scanlines it reads.
                JSAMPROW row = const_cast<JSAMPROW>(pixels + static_cast<size_t>(cinfo.next_scanline) * stride);
                jpeg_write_scanlines(&cinfo, &row, 1);
            }

            jpeg_finish_compress(&cinfo);
            jpeg_destroy_compress(&cinfo);
            return true;
        }

        AZ::Outcome<sensor_msgs::msg::CompressedImage, AZStd::string> CompressJpeg(
            const sensor_msgs::msg::Image& image, const Settings& settings)
        {
            const int quality = AZ::GetClamp(settings.m_jpegQuality, Settings::MinJpegQuality, Settings::MaxJpegQuality);

            // Selected on the source encoding: what libjpeg is fed, and what a consumer gets back from decoding.
            const uint8_t* pixels = image.data.data();
            size_t stride = image.step;
            std::vector<uint8_t> repacked;
            int channels = 0;
            J_COLOR_SPACE colorSpace = JCS_UNKNOWN;
            const char* decodedEncoding = nullptr;

            if (image.encoding == "rgb8")
            {
                channels = 3;
                colorSpace = JCS_RGB;
                decodedEncoding = "bgr8";
            }
            else if (image.encoding == "rgba8")
            {
                repacked = DropAlpha(image);
                pixels = repacked.data();
                stride = static_cast<size_t>(image.width) * 3;
                channels = 3;
                colorSpace = JCS_RGB;
                decodedEncoding = "bgr8";
            }
            else if (image.encoding == "mono8")
            {
                channels = 1;
                colorSpace = JCS_GRAYSCALE;
                decodedEncoding = "mono8";
            }
            else
            {
                return AZ::Failure(AZStd::string::format(
                    "JPEG cannot store %s; it is limited to 8 bits per sample. Use PNG instead.", image.encoding.c_str()));
            }

            unsigned char* jpegBuffer = nullptr;
            unsigned long jpegSize = 0;
            if (!EncodeJpeg(pixels, stride, image.width, image.height, channels, colorSpace, quality, &jpegBuffer, &jpegSize))
            {
                return AZ::Failure(AZStd::string("JPEG compression failed"));
            }

            sensor_msgs::msg::CompressedImage result;
            result.header = image.header;
            result.data.assign(jpegBuffer, jpegBuffer + jpegSize);
            result.format = AZStd::string::format("%s; jpeg compressed %s", image.encoding.c_str(), decodedEncoding).c_str();
            free(jpegBuffer);
            return AZ::Success(AZStd::move(result));
        }
#endif // ROS2SENSORS_WITH_JPEG

        AZ::Outcome<sensor_msgs::msg::CompressedImage, AZStd::string> CompressPng(
            const sensor_msgs::msg::Image& image, const Settings& settings)
        {
            const int level =
                AZ::GetClamp(settings.m_pngCompressionLevel, Settings::MinPngCompressionLevel, Settings::MaxPngCompressionLevel);

            // Selected on the source encoding: what libpng is fed, and what a consumer gets back from decoding.
            const uint8_t* pixels = image.data.data();
            size_t stride = image.step;
            std::vector<uint8_t> repacked;
            int colorType = 0;
            int bitDepth = 8;
            const char* decodedEncoding = nullptr;

            if (image.encoding == "rgb8")
            {
                colorType = PNG_COLOR_TYPE_RGB;
                decodedEncoding = "bgr8";
            }
            else if (image.encoding == "rgba8")
            {
                colorType = PNG_COLOR_TYPE_RGBA;
                decodedEncoding = "bgra8";
            }
            else if (image.encoding == "mono8")
            {
                colorType = PNG_COLOR_TYPE_GRAY;
                decodedEncoding = "mono8";
            }
            else if (image.encoding == "mono16")
            {
                repacked = ToBigEndianMono16(image);
                pixels = repacked.data();
                stride = static_cast<size_t>(image.width) * sizeof(uint16_t);
                colorType = PNG_COLOR_TYPE_GRAY;
                bitDepth = 16;
                decodedEncoding = "mono16";
            }
            else
            {
                return AZ::Failure(AZStd::string::format("PNG cannot store %s", image.encoding.c_str()));
            }

            sensor_msgs::msg::CompressedImage result;
            result.header = image.header;
            result.data.reserve(image.data.size() / 2);
            if (!EncodePng(pixels, stride, image.width, image.height, colorType, bitDepth, level, &result.data))
            {
                return AZ::Failure(AZStd::string("PNG compression failed"));
            }
            result.format = AZStd::string::format("%s; png compressed %s", image.encoding.c_str(), decodedEncoding).c_str();
            return AZ::Success(AZStd::move(result));
        }
    } // namespace

    void Settings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<Settings>()
                ->Version(0)
                ->Field("Codec", &Settings::m_codec)
                ->Field("JpegQuality", &Settings::m_jpegQuality)
                ->Field("PngCompressionLevel", &Settings::m_pngCompressionLevel);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<Settings>("Compression Settings", "Codec and its parameters")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &Settings::m_codec,
                        "Codec",
                        "Codec for 8-bit images. Depth and mono16 require PNG; JPEG cannot store them and publishes nothing.")
                    ->EnumAttribute(Codec::Jpeg, "JPEG (lossy, 8-bit only)")
                    ->EnumAttribute(Codec::Png, "PNG (lossless)")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider,
                        &Settings::m_jpegQuality,
                        "JPEG Quality",
                        "JPEG quality, from smallest file to best image")
                    ->Attribute(AZ::Edit::Attributes::Min, MinJpegQuality)
                    ->Attribute(AZ::Edit::Attributes::Max, MaxJpegQuality)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &Settings::GetJpegVisibility)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider,
                        &Settings::m_pngCompressionLevel,
                        "PNG Compression Level",
                        "zlib compression level, from fastest to smallest file")
                    ->Attribute(AZ::Edit::Attributes::Min, MinPngCompressionLevel)
                    ->Attribute(AZ::Edit::Attributes::Max, MaxPngCompressionLevel)
                     ->Attribute(AZ::Edit::Attributes::Visibility, &Settings::GetPngVisibility);
            }
        }
    }

    AZ::Crc32 Settings::GetJpegVisibility() const
    {
        return m_codec == Codec::Jpeg ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    AZ::Crc32 Settings::GetPngVisibility() const
    {
        return m_codec == Codec::Png ? AZ::Edit::PropertyVisibility::Show : AZ::Edit::PropertyVisibility::Hide;
    }

    bool IsCodecAvailable([[maybe_unused]] Codec codec)
    {
#ifndef ROS2SENSORS_WITH_JPEG
        if (codec == Codec::Jpeg)
        {
            return false;
        }
#endif
        return true;
    }

    AZ::Outcome<sensor_msgs::msg::CompressedImage, AZStd::string> Compress(
        const sensor_msgs::msg::Image& image, const Settings& settings)
    {
        const auto layout = GetValidatedLayout(image);
        if (!layout.IsSuccess())
        {
            return AZ::Failure(layout.GetError());
        }

        switch (settings.m_codec)
        {
        case Codec::Jpeg:
#ifdef ROS2SENSORS_WITH_JPEG
            // Encodings wider than 8 bits fail rather than falling back: nothing is published, instead of
            // publishing a codec the user did not ask for. CompressJpeg rejects them.
            return CompressJpeg(image, settings);
#else
            return AZ::Failure(AZStd::string("JPEG support was not compiled in; libjpeg was not found at configure time"));
#endif
        case Codec::Png:
            return CompressPng(image, settings);
        }
        return AZ::Failure(AZStd::string("Unknown codec"));
    }
} // namespace ROS2Sensors::ImageCompression
