/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Camera/PostProcessing/ROS2ImageEncodingConversionComponent.h>

#include <AzTest/AzTest.h>
#include <cmath>
#include <cstring>
#include <limits>

namespace ROS2Sensors
{
    namespace
    {
        using ImageEncoding = CameraUtils::ImageEncoding;

        //! A 32FC1 depth image, one row of `depths` per row of the image, optionally row-padded so the
        //! conversions are exercised with a step larger than a packed row.
        sensor_msgs::msg::Image MakeDepthImage(const AZStd::vector<float>& depths, uint32_t width, uint32_t rowPadding = 0)
        {
            const uint32_t height = static_cast<uint32_t>(depths.size()) / width;
            sensor_msgs::msg::Image image;
            image.encoding = CameraUtils::ImageEncodingNames.at(ImageEncoding::Depth32FC1);
            image.width = width;
            image.height = height;
            image.step = static_cast<uint32_t>(width * sizeof(float)) + rowPadding;
            image.data.assign(static_cast<size_t>(image.step) * height, 0);

            for (uint32_t row = 0; row < height; ++row)
            {
                for (uint32_t column = 0; column < width; ++column)
                {
                    const float depth = depths[static_cast<size_t>(row) * width + column];
                    std::memcpy(
                        image.data.data() + static_cast<size_t>(row) * image.step + column * sizeof(float), &depth, sizeof(depth));
                }
            }
            return image;
        }

        template<typename TargetSample>
        AZStd::vector<TargetSample> ReadSamples(const sensor_msgs::msg::Image& image)
        {
            AZStd::vector<TargetSample> samples;
            for (uint32_t row = 0; row < image.height; ++row)
            {
                for (uint32_t column = 0; column < image.width; ++column)
                {
                    TargetSample sample{};
                    std::memcpy(
                        &sample,
                        image.data.data() + static_cast<size_t>(row) * image.step + column * sizeof(TargetSample),
                        sizeof(sample));
                    samples.push_back(sample);
                }
            }
            return samples;
        }

        EncodingConversion DepthToMono16(float scaleFactor)
        {
            return EncodingConversion{ ImageEncoding::Depth32FC1, ImageEncoding::Mono16, scaleFactor };
        }
    } // namespace

    TEST(EncodingConversionTest, Depth32FC1ToMono16ScalesSamples)
    {
        // 1000 is the conventional metres-to-millimetres factor the compression component defaults to.
        auto image = MakeDepthImage({ 0.5f, 1.0f, 2.5f, 10.0f }, 2);
        EXPECT_TRUE(ApplyEncodingConversion(image, DepthToMono16(1000.0f)));

        EXPECT_EQ(image.encoding, CameraUtils::ImageEncodingNames.at(ImageEncoding::Mono16));
        EXPECT_EQ(image.step, image.width * sizeof(uint16_t));
        EXPECT_EQ(image.data.size(), image.step * image.height);
        EXPECT_THAT(ReadSamples<uint16_t>(image), ::testing::ElementsAre(500, 1000, 2500, 10000));
    }

    TEST(EncodingConversionTest, Depth32FC1ToMono16HandlesRowPadding)
    {
        auto image = MakeDepthImage({ 1.0f, 2.0f, 3.0f, 4.0f }, 2, 8);
        EXPECT_TRUE(ApplyEncodingConversion(image, DepthToMono16(1000.0f)));

        EXPECT_EQ(image.step, image.width * sizeof(uint16_t));
        EXPECT_THAT(ReadSamples<uint16_t>(image), ::testing::ElementsAre(1000, 2000, 3000, 4000));
    }

    TEST(EncodingConversionTest, Depth32FC1ToMono16MapsUnmeasuredSamplesToZero)
    {
        // Not finite, negative, and past the target range all mean "no reading", not "very far away".
        const float infinity = std::numeric_limits<float>::infinity();
        const float notANumber = std::numeric_limits<float>::quiet_NaN();
        auto image = MakeDepthImage({ infinity, -infinity, notANumber, -1.0f, 100.0f, 1.0f }, 6);

        EXPECT_TRUE(ApplyEncodingConversion(image, DepthToMono16(1000.0f)));
        // 100 m scales to 100000, past the 65535 a uint16 can hold.
        EXPECT_THAT(ReadSamples<uint16_t>(image), ::testing::ElementsAre(0, 0, 0, 0, 0, 1000));
    }

    TEST(EncodingConversionTest, Depth32FC1ToMono8Quantizes)
    {
        auto image = MakeDepthImage({ 0.0f, 1.0f, 2.0f, 300.0f }, 4);
        EXPECT_TRUE(ApplyEncodingConversion(image, EncodingConversion{ ImageEncoding::Depth32FC1, ImageEncoding::Mono8, 1.0f }));

        EXPECT_EQ(image.encoding, CameraUtils::ImageEncodingNames.at(ImageEncoding::Mono8));
        EXPECT_EQ(image.step, image.width * sizeof(uint8_t));
        EXPECT_THAT(ReadSamples<uint8_t>(image), ::testing::ElementsAre(0, 1, 2, 0));
    }

    TEST(EncodingConversionTest, Rgba8ToRgb8DropsAlpha)
    {
        sensor_msgs::msg::Image image;
        image.encoding = CameraUtils::ImageEncodingNames.at(ImageEncoding::RGBA8);
        image.width = 2;
        image.height = 1;
        image.step = image.width * 4;
        image.data = { 1, 2, 3, 255, 4, 5, 6, 255 };

        EXPECT_TRUE(ApplyEncodingConversion(image, EncodingConversion{ ImageEncoding::RGBA8, ImageEncoding::RGB8, 1.0f }));

        EXPECT_EQ(image.encoding, CameraUtils::ImageEncodingNames.at(ImageEncoding::RGB8));
        EXPECT_EQ(image.step, image.width * 3);
        EXPECT_EQ(image.data, std::vector<uint8_t>({ 1, 2, 3, 4, 5, 6 }));
    }

    TEST(EncodingConversionTest, MismatchedInputEncodingIsLeftAlone)
    {
        auto image = MakeDepthImage({ 1.0f, 2.0f }, 2);
        const auto original = image;

        // The image is depth, the conversion expects rgba8.
        EXPECT_FALSE(ApplyEncodingConversion(image, EncodingConversion{ ImageEncoding::RGBA8, ImageEncoding::RGB8, 1.0f }));
        EXPECT_EQ(image.encoding, original.encoding);
        EXPECT_EQ(image.data, original.data);
    }

    TEST(EncodingConversionTest, UnsupportedConversionIsLeftAlone)
    {
        auto image = MakeDepthImage({ 1.0f, 2.0f }, 2);
        const auto original = image;

        // There is no 32FC1 to rgb8 entry in the conversion table.
        EXPECT_FALSE(ApplyEncodingConversion(image, EncodingConversion{ ImageEncoding::Depth32FC1, ImageEncoding::RGB8, 1.0f }));
        EXPECT_EQ(image.encoding, original.encoding);
        EXPECT_EQ(image.data, original.data);
    }

    TEST(EncodingConversionTest, UnknownEncodingNameIsLeftAlone)
    {
        sensor_msgs::msg::Image image;
        image.encoding = "bgr8";
        image.width = 1;
        image.height = 1;
        image.step = 3;
        image.data = { 1, 2, 3 };

        EXPECT_FALSE(ApplyEncodingConversion(image, DepthToMono16(1000.0f)));
        EXPECT_EQ(image.encoding, "bgr8");
    }
} // namespace ROS2Sensors