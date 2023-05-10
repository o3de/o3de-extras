/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraSensorDescription.h"
#include <AzCore/Math/MatrixUtils.h>

namespace ROS2
{
    CameraSensorDescription::CameraSensorDescription(
        const AZStd::string& cameraName, const CameraConfiguration& configuration, const SensorConfiguration& sensorConfiguration)
        : m_cameraConfiguration(configuration)
        , m_sensorConfiguration(sensorConfiguration)
        , m_cameraName(cameraName)
        , m_aspectRatio(static_cast<float>(configuration.m_width) / static_cast<float>(configuration.m_height))
        , m_viewToClipMatrix(MakeViewToClipMatrix())
        , m_cameraIntrinsics(MakeCameraIntrinsics())
    {
        ValidateParameters();
    }

    AZ::Matrix4x4 CameraSensorDescription::MakeViewToClipMatrix() const
    {
        const float nearDist = 0.1f, farDist = 100.0f;
        AZ::Matrix4x4 localViewToClipMatrix;
        AZ::MakePerspectiveFovMatrixRH(
            localViewToClipMatrix, AZ::DegToRad(m_cameraConfiguration.m_verticalFieldOfViewDeg), m_aspectRatio, nearDist, farDist, true);
        return localViewToClipMatrix;
    }

    void CameraSensorDescription::ValidateParameters() const
    {
        AZ_Assert(
            m_cameraConfiguration.m_verticalFieldOfViewDeg > 0.0f && m_cameraConfiguration.m_verticalFieldOfViewDeg < 180.0f,
            "Vertical fov should be in range 0.0 < FoV < 180.0 degrees");
        AZ_Assert(!m_cameraName.empty(), "Camera name cannot be empty");
    }

    AZStd::array<double, 9> CameraSensorDescription::MakeCameraIntrinsics() const
    {
        /* Intrinsic camera matrix of the camera image is being created here
           It is based on other parameters available in the structure - they must be initialized before this function is called
            Matrix is row-major and has the following form:
            [fx  0 cx]
            [ 0 fy cy]
            [ 0  0  1]
           Projects 3D points in the camera coordinate frame to 2D pixel
           coordinates using the focal lengths (fx, fy) and principal point
           (cx, cy).
       */
        const auto w = static_cast<double>(m_cameraConfiguration.m_width);
        const auto h = static_cast<double>(m_cameraConfiguration.m_height);
        const double verticalFieldOfView = AZ::DegToRad(m_cameraConfiguration.m_verticalFieldOfViewDeg);
        const double horizontalFoV = 2.0 * AZStd::atan(AZStd::tan(verticalFieldOfView / 2.0) * m_aspectRatio);
        const double focalLengthX = w / (2.0 * AZStd::tan(horizontalFoV / 2.0));
        const double focalLengthY = h / (2.0 * AZStd::tan(verticalFieldOfView / 2.0));
        return { focalLengthX, 0.0, w / 2.0, 0.0, focalLengthY, h / 2.0, 0.0, 0.0, 1.0 };
    }
} // namespace ROS2
