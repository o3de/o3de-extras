/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CameraSensorDescription.h"
#include "CameraUtilities.h"
#include <AzCore/Math/MatrixUtils.h>

namespace ROS2
{
    CameraSensorDescription::CameraSensorDescription(
        const AZStd::string& cameraName,
        const AZStd::string& effectiveNamespace,
        const CameraSensorConfiguration& configuration,
        const SensorConfiguration& sensorConfiguration)
        : m_cameraConfiguration(configuration)
        , m_sensorConfiguration(sensorConfiguration)
        , m_cameraName(cameraName)
        , m_cameraNamespace(effectiveNamespace)
        , m_viewToClipMatrix(CameraUtils::MakeClipMatrix(
              m_cameraConfiguration.m_width,
              m_cameraConfiguration.m_height,
              m_cameraConfiguration.m_verticalFieldOfViewDeg,
              m_cameraConfiguration.m_nearClipDistance,
              m_cameraConfiguration.m_farClipDistance))
        , m_cameraIntrinsics(CameraUtils::MakeCameraIntrinsics(
              m_cameraConfiguration.m_width, m_cameraConfiguration.m_height, m_cameraConfiguration.m_verticalFieldOfViewDeg))
    {
        ValidateParameters();
    }

    void CameraSensorDescription::ValidateParameters() const
    {
        AZ_Assert(
            m_cameraConfiguration.m_verticalFieldOfViewDeg > 0.0f && m_cameraConfiguration.m_verticalFieldOfViewDeg < 180.0f,
            "Vertical fov should be in range 0.0 < FoV < 180.0 degrees");
        AZ_Assert(
            m_cameraConfiguration.m_width > 0 && m_cameraConfiguration.m_height > 0, "Camera resolution dimensions should be above zero");
        AZ_Assert(!m_cameraName.empty(), "Camera name cannot be empty");
        AZ_Assert(m_cameraConfiguration.m_nearClipDistance > 0.0f, "Near clip distance should be greater than zero");
        AZ_Assert(m_cameraConfiguration.m_farClipDistance > m_cameraConfiguration.m_nearClipDistance , "Far clip distance should be greater than the near plane distance");
        AZ_Assert(
            m_cameraConfiguration.m_farClipDistance > m_cameraConfiguration.m_nearClipDistance,
            "Far clip distance should be greater than near clip distance");
    }

} // namespace ROS2
