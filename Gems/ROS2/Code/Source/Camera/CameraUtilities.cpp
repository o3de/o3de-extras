/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/MatrixUtils.h>

namespace ROS2::CameraUtils
{
    float GetAspectRatio(float width, float height)
    {
        return width / height;
    };

    AZ::Matrix3x3 MakeCameraIntrinsics(int width, int height, float verticalFieldOfViewDeg)
    {
        const auto w = static_cast<float>(width);
        const auto h = static_cast<float>(height);
        const float verticalFieldOfView = AZ::DegToRad(verticalFieldOfViewDeg);
        const float horizontalFoV = 2.0 * AZStd::atan(AZStd::tan(verticalFieldOfView / 2.0) * GetAspectRatio(width, height));
        const float focalLengthX = w / (2.0 * AZStd::tan(horizontalFoV / 2.0));
        const float focalLengthY = h / (2.0 * AZStd::tan(verticalFieldOfView / 2.0));
        return AZ::Matrix3x3::CreateFromRows({ focalLengthX, 0.f, w / 2.f }, { 0.f, focalLengthY, h / 2.f }, { 0.f, 0.f, 1.f });
    }

    AZ::Matrix4x4 MakeClipMatrix(int width, int height, float verticalFieldOfViewDeg, float nearDist, float farDist)
    {
        AZ::Matrix4x4 localViewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(
            localViewToClipMatrix,
            AZ::DegToRad(verticalFieldOfViewDeg),
            CameraUtils::GetAspectRatio(width, height),
            nearDist,
            farDist,
            true);
        return localViewToClipMatrix;
    }

} // namespace ROS2::CameraUtils