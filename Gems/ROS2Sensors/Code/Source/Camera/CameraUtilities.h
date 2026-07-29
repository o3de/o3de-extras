/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Matrix3x3.h>

//! Namespace contains utility functions for camera.
namespace ROS2Sensors::CameraUtils
{
    //! Available image encodings
    //! @note Append new entries; the values are serialized by EncodingConversion.
    enum class ImageEncoding : AZ::u8
    {
        RGBA8,
        RGB8,
        Mono8,
        Mono16,
        Depth32FC1,
    };
    const AZStd::unordered_map<ImageEncoding, const char*> ImageEncodingNames = {
        { ImageEncoding::RGBA8, "rgba8" },   { ImageEncoding::RGB8, "rgb8" },     { ImageEncoding::Mono8, "mono8" },
        { ImageEncoding::Mono16, "mono16" }, { ImageEncoding::Depth32FC1, "32FC1" },
    };
    const AZStd::unordered_map<AZStd::string, ImageEncoding> ImageEncodingFromName = {
        { "rgba8", ImageEncoding::RGBA8 },   { "rgb8", ImageEncoding::RGB8 },     { "mono8", ImageEncoding::Mono8 },
        { "mono16", ImageEncoding::Mono16 }, { "32FC1", ImageEncoding::Depth32FC1 },
    };

    //! Function computes aspect ratio of the image.
    //! @param width Width of the image in pixels
    //! @param height Height of the image in pixels.
    //! @return Aspect ratio of the image.
    float GetAspectRatio(float width, float height);

    //! Function computes 3x3 projection matrix (pinhole model) from camera config.
    //! @param height Height of the image in pixels.
    //! @param width Width of the image in pixels
    //! @param verticalFieldOfViewDeg  Vertical field of view of the camera in degrees.
    //! @return projection matrix for computer vision applications.
    AZ::Matrix3x3 MakeCameraIntrinsics(int width, int height, float verticalFieldOfViewDeg);

    //! Function computes 4x4 projection matrix (frustum model) from camera config.
    //! @param height Height of the image in pixels.
    //! @param height Height of the image in pixels.
    //! @param verticalFieldOfViewDeg Vertical field of view of the camera in degrees.
    //! @param farDist Far clipping plane distance in meters.
    //! @param nearDist Near clipping plane distance in meters.
    //! @return projection matrix for the rendering.
    AZ::Matrix4x4 MakeClipMatrix(int width, int height, float verticalFieldOfViewDeg, float nearDist = 0.1f, float farDist = 100.0f);
} // namespace ROS2Sensors::CameraUtils
