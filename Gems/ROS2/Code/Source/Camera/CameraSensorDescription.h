/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROS2/Sensor/SensorConfiguration.h>

#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/std/containers/array.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Data structure containing camera parameters for fov and resolution.
    struct CameraConfiguration
    {
        float m_verticalFieldOfViewDeg = 90.0f; //!< Vertical field of view of camera sensor.
        int m_width = 640; //!< Camera image width in pixels.
        int m_height = 480; //!< Camera image height in pixels.
    };

    //! Structure containing all information required to create the camera sensor.
    struct CameraSensorDescription
    {
        //! Type of camera channel.
        enum class CameraChannelType
        {
            RGB = 0,
            DEPTH = 1
        };

        //! Constructor to create the description
        //! @param cameraName - name of the camera; used to differentiate cameras in a multi-camera setup.
        //! @param configuration - configuration structure for the camera, defining its characteristics.
        //! @param sensorConfiguration - generic configuration for this sensor.
        CameraSensorDescription(
            const AZStd::string& cameraName, const CameraConfiguration& configuration, const SensorConfiguration& sensorConfiguration);

        const CameraConfiguration m_cameraConfiguration; //!< Configuration of the camera.
        const SensorConfiguration m_sensorConfiguration; //!< Generic sensor configuration.
        const AZStd::string m_cameraName; //!< Camera name to differentiate cameras in a multi-camera setup.

        const float m_aspectRatio; //!< Camera image aspect ratio; equal to (width / height).
        const AZ::Matrix4x4 m_viewToClipMatrix; //!< Camera view to clip space transform matrix; derived from other parameters.
        const AZStd::array<double, 9> m_cameraIntrinsics; //!< Camera intrinsics; derived from other parameters.

    private:
        AZ::Matrix4x4 MakeViewToClipMatrix() const;
        AZStd::array<double, 9> MakeCameraIntrinsics() const;
        void ValidateParameters() const;
    };
} // namespace ROS2
