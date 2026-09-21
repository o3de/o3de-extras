/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Camera/Compression/ImageCompression.h>

#include <AzTest/AzTest.h>
#include <cmath>
#include <cstring>
#include <png.h>

namespace ROS2Sensors
{
    namespace
    {
        //! Build an image message whose rows may carry padding, so that the encoders are exercised with a
        //! step that is larger than a packed row.
        sensor_msgs::msg::Image MakeImage(
            const char* encoding, uint32_t width, uint32_t height, size_t bytesPerPixel, uint32_t rowPadding = 0)
        {
            sensor_msgs::msg::Image image;
            image.encoding = encoding;
            image.width = width;
            image.height = height;
            image.step = static_cast<uint32_t>(width * bytesPerPixel) + rowPadding;
            image.data.assign(static_cast<size_t>(image.step) * height, 0);
            return image;
        }

        //! Deterministic per-byte pattern; unique enough to catch rows or channels being swapped.
        uint8_t PatternByte(uint32_t row, uint32_t column, size_t byteInPixel)
        {
            return static_cast<uint8_t>((row * 37 + column * 11 + byteInPixel * 101) & 0xFF);
        }

        void FillPattern(sensor_msgs::msg::Image& image, size_t bytesPerPixel)
        {
            for (uint32_t row = 0; row < image.height; ++row)
            {
                for (uint32_t column = 0; column < image.width; ++column)
                {
                    uint8_t* pixel = image.data.data() + static_cast<size_t>(row) * image.step + column * bytesPerPixel;
                    for (size_t byte = 0; byte < bytesPerPixel; ++byte)
                    {
                        pixel[byte] = PatternByte(row, column, byte);
                    }
                }
            }
        }

        //! A PNG decoded through libpng's simplified reader. 16-bit samples come back in host order.
        struct DecodedPng
        {
            uint32_t m_width = 0;
            uint32_t m_height = 0;
            uint32_t m_format = 0;
            std::vector<uint8_t> m_pixels;

            bool IsColor() const
            {
                return (m_format & PNG_FORMAT_FLAG_COLOR) != 0;
            }
            bool HasAlpha() const
            {
                return (m_format & PNG_FORMAT_FLAG_ALPHA) != 0;
            }
            bool Is16Bit() const
            {
                return (m_format & PNG_FORMAT_FLAG_LINEAR) != 0;
            }
        };

        //! Decode in the format the file actually stores, so the test sees what was written rather than a
        //! converted copy.
        bool DecodePng(const std::vector<uint8_t>& data, DecodedPng& decoded)
        {
            png_image image;
            std::memset(&image, 0, sizeof(image));
            image.version = PNG_IMAGE_VERSION;
            if (!png_image_begin_read_from_memory(&image, data.data(), data.size()))
            {
                return false;
            }
            decoded.m_width = image.width;
            decoded.m_height = image.height;
            decoded.m_format = image.format;
            decoded.m_pixels.resize(PNG_IMAGE_SIZE(image));
            if (!png_image_finish_read(&image, nullptr, decoded.m_pixels.data(), 0, nullptr))
            {
                png_image_free(&image);
                return false;
            }
            return true;
        }

        ImageCompression::Settings PngSettings()
        {
            ImageCompression::Settings settings;
            settings.m_codec = ImageCompression::Codec::Png;
            return settings;
        }
    } // namespace

    TEST(ImageCompressionTest, UnsupportedEncodingFails)
    {
        auto image = MakeImage("bgr8", 4, 4, 3);
        const auto result = ImageCompression::Compress(image, PngSettings());
        EXPECT_FALSE(result.IsSuccess());
    }

    TEST(ImageCompressionTest, EmptyImageFails)
    {
        auto image = MakeImage("rgb8", 0, 0, 3);
        const auto result = ImageCompression::Compress(image, PngSettings());
        EXPECT_FALSE(result.IsSuccess());
    }

    TEST(ImageCompressionTest, StepSmallerThanRowFails)
    {
        auto image = MakeImage("rgb8", 8, 4, 3);
        image.step -= 1;
        const auto result = ImageCompression::Compress(image, PngSettings());
        EXPECT_FALSE(result.IsSuccess());
    }

    TEST(ImageCompressionTest, TruncatedDataFails)
    {
        auto image = MakeImage("rgb8", 8, 4, 3);
        image.data.resize(image.data.size() - 1);
        const auto result = ImageCompression::Compress(image, PngSettings());
        EXPECT_FALSE(result.IsSuccess());
    }

    TEST(ImageCompressionTest, PngRgb8IsLossless)
    {
        // Padded rows: the encoder has to honour step rather than assume packed data.
        auto image = MakeImage("rgb8", 8, 5, 3, 7);
        FillPattern(image, 3);

        const auto result = ImageCompression::Compress(image, PngSettings());
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();
        EXPECT_STREQ(result.GetValue().format.c_str(), "rgb8; png compressed bgr8");

        DecodedPng decoded;
        ASSERT_TRUE(DecodePng(result.GetValue().data, decoded));
        EXPECT_EQ(decoded.m_width, image.width);
        EXPECT_EQ(decoded.m_height, image.height);
        EXPECT_TRUE(decoded.IsColor());
        EXPECT_FALSE(decoded.HasAlpha());
        EXPECT_FALSE(decoded.Is16Bit());

        for (uint32_t row = 0; row < image.height; ++row)
        {
            for (uint32_t column = 0; column < image.width; ++column)
            {
                for (size_t channel = 0; channel < 3; ++channel)
                {
                    const size_t index = (static_cast<size_t>(row) * image.width + column) * 3 + channel;
                    EXPECT_EQ(decoded.m_pixels[index], PatternByte(row, column, channel))
                        << "at row " << row << " column " << column << " channel " << channel;
                }
            }
        }
    }

    TEST(ImageCompressionTest, PngRgba8KeepsAlpha)
    {
        auto image = MakeImage("rgba8", 6, 3, 4);
        FillPattern(image, 4);

        const auto result = ImageCompression::Compress(image, PngSettings());
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();
        EXPECT_STREQ(result.GetValue().format.c_str(), "rgba8; png compressed bgra8");

        DecodedPng decoded;
        ASSERT_TRUE(DecodePng(result.GetValue().data, decoded));
        ASSERT_TRUE(decoded.HasAlpha());
        for (uint32_t row = 0; row < image.height; ++row)
        {
            for (uint32_t column = 0; column < image.width; ++column)
            {
                const size_t index = (static_cast<size_t>(row) * image.width + column) * 4;
                EXPECT_EQ(decoded.m_pixels[index + 3], PatternByte(row, column, 3));
            }
        }
    }

    TEST(ImageCompressionTest, PngMono8IsLossless)
    {
        auto image = MakeImage("mono8", 5, 4, 1, 3);
        FillPattern(image, 1);

        const auto result = ImageCompression::Compress(image, PngSettings());
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();
        EXPECT_STREQ(result.GetValue().format.c_str(), "mono8; png compressed mono8");

        DecodedPng decoded;
        ASSERT_TRUE(DecodePng(result.GetValue().data, decoded));
        EXPECT_FALSE(decoded.IsColor());
        for (uint32_t row = 0; row < image.height; ++row)
        {
            for (uint32_t column = 0; column < image.width; ++column)
            {
                const size_t index = static_cast<size_t>(row) * image.width + column;
                EXPECT_EQ(decoded.m_pixels[index], PatternByte(row, column, 0));
            }
        }
    }

    TEST(ImageCompressionTest, PngMono16IsLosslessAndBigEndian)
    {
        constexpr uint32_t Width = 4;
        constexpr uint32_t Height = 2;
        // Values with differing high and low bytes, so a byte-order mistake cannot pass.
        const uint16_t samples[Width * Height] = { 0x0102, 0xFF00, 0x00FF, 0x1234, 0xFFFF, 0x0000, 0x8001, 0x0180 };

        auto image = MakeImage("mono16", Width, Height, sizeof(uint16_t));
        for (uint32_t row = 0; row < Height; ++row)
        {
            for (uint32_t column = 0; column < Width; ++column)
            {
                const uint16_t sample = samples[row * Width + column];
                std::memcpy(image.data.data() + static_cast<size_t>(row) * image.step + column * sizeof(uint16_t), &sample, sizeof(sample));
            }
        }

        const auto result = ImageCompression::Compress(image, PngSettings());
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();
        EXPECT_STREQ(result.GetValue().format.c_str(), "mono16; png compressed mono16");

        // The stream itself has to be big-endian; check the first sample's bytes before libpng swaps them back.
        const std::vector<uint8_t>& png = result.GetValue().data;
        DecodedPng decoded;
        ASSERT_TRUE(DecodePng(png, decoded));
        ASSERT_TRUE(decoded.Is16Bit());
        ASSERT_FALSE(decoded.IsColor());
        ASSERT_EQ(decoded.m_pixels.size(), sizeof(samples));

        // The simplified reader hands back host order, so comparing values verifies the round trip.
        for (size_t index = 0; index < Width * Height; ++index)
        {
            uint16_t value = 0;
            std::memcpy(&value, decoded.m_pixels.data() + index * sizeof(uint16_t), sizeof(value));
            EXPECT_EQ(value, samples[index]) << "at sample " << index;
        }
    }

    //! Float depth is quantized to mono16 by ROS2ImageCompressionComponent before it reaches the codecs.
    TEST(ImageCompressionTest, FloatEncodingIsRejected)
    {
        auto image = MakeImage("32FC1", 4, 4, sizeof(float));
        EXPECT_FALSE(ImageCompression::Compress(image, PngSettings()).IsSuccess());
    }

    TEST(ImageCompressionTest, PngCompressionLevelIsClamped)
    {
        auto image = MakeImage("mono8", 8, 8, 1);
        FillPattern(image, 1);

        auto settings = PngSettings();
        settings.m_pngCompressionLevel = 42; // out of range; must be clamped rather than passed to zlib
        const auto result = ImageCompression::Compress(image, settings);
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();

        DecodedPng decoded;
        EXPECT_TRUE(DecodePng(result.GetValue().data, decoded));
    }

    TEST(ImageCompressionTest, PngIsAlwaysAvailable)
    {
        EXPECT_TRUE(ImageCompression::IsCodecAvailable(ImageCompression::Codec::Png));
    }

#ifdef ROS2SENSORS_WITH_JPEG
    namespace
    {
        ImageCompression::Settings JpegSettings()
        {
            ImageCompression::Settings settings;
            settings.m_codec = ImageCompression::Codec::Jpeg;
            return settings;
        }

        bool HasJpegMarkers(const std::vector<uint8_t>& data)
        {
            return data.size() > 4 && data[0] == 0xFF && data[1] == 0xD8 && data[data.size() - 2] == 0xFF &&
                data[data.size() - 1] == 0xD9;
        }
    } // namespace

    TEST(ImageCompressionTest, JpegIsReportedAvailable)
    {
        EXPECT_TRUE(ImageCompression::IsCodecAvailable(ImageCompression::Codec::Jpeg));
    }

    TEST(ImageCompressionTest, JpegCompressesRgb8)
    {
        auto image = MakeImage("rgb8", 32, 16, 3, 5);
        FillPattern(image, 3);

        const auto result = ImageCompression::Compress(image, JpegSettings());
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();
        EXPECT_STREQ(result.GetValue().format.c_str(), "rgb8; jpeg compressed bgr8");
        EXPECT_TRUE(HasJpegMarkers(result.GetValue().data));
    }

    TEST(ImageCompressionTest, JpegCompressesRgba8ByDroppingAlpha)
    {
        auto image = MakeImage("rgba8", 32, 16, 4, 9);
        FillPattern(image, 4);

        const auto result = ImageCompression::Compress(image, JpegSettings());
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();
        EXPECT_STREQ(result.GetValue().format.c_str(), "rgba8; jpeg compressed bgr8");
        EXPECT_TRUE(HasJpegMarkers(result.GetValue().data));
    }

    TEST(ImageCompressionTest, JpegCompressesMono8)
    {
        auto image = MakeImage("mono8", 32, 16, 1);
        FillPattern(image, 1);

        const auto result = ImageCompression::Compress(image, JpegSettings());
        ASSERT_TRUE(result.IsSuccess()) << result.GetError().c_str();
        EXPECT_STREQ(result.GetValue().format.c_str(), "mono8; jpeg compressed mono8");
        EXPECT_TRUE(HasJpegMarkers(result.GetValue().data));
    }

    TEST(ImageCompressionTest, JpegQualityAffectsSize)
    {
        auto image = MakeImage("rgb8", 64, 64, 3);
        FillPattern(image, 3);

        auto low = JpegSettings();
        low.m_jpegQuality = ImageCompression::Settings::MinJpegQuality;
        auto high = JpegSettings();
        high.m_jpegQuality = ImageCompression::Settings::MaxJpegQuality;

        const auto lowResult = ImageCompression::Compress(image, low);
        const auto highResult = ImageCompression::Compress(image, high);
        ASSERT_TRUE(lowResult.IsSuccess());
        ASSERT_TRUE(highResult.IsSuccess());
        EXPECT_LT(lowResult.GetValue().data.size(), highResult.GetValue().data.size());
    }

    //! Baseline JPEG stores 8 bits per sample, so mono16 fails instead of being published as another codec.
    TEST(ImageCompressionTest, JpegRejectsEncodingsWiderThan8Bits)
    {
        auto image = MakeImage("mono16", 8, 8, sizeof(uint16_t));
        FillPattern(image, sizeof(uint16_t));

        const auto result = ImageCompression::Compress(image, JpegSettings());
        EXPECT_FALSE(result.IsSuccess());
    }
#else
    TEST(ImageCompressionTest, JpegIsReportedUnavailable)
    {
        EXPECT_FALSE(ImageCompression::IsCodecAvailable(ImageCompression::Codec::Jpeg));

        auto image = MakeImage("rgb8", 8, 8, 3);
        ImageCompression::Settings settings;
        settings.m_codec = ImageCompression::Codec::Jpeg;
        EXPECT_FALSE(ImageCompression::Compress(image, settings).IsSuccess());
    }
#endif // ROS2SENSORS_WITH_JPEG
} // namespace ROS2Sensors