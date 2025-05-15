/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>

namespace ROS2Sensors
{
    //! Interface that allows to get and set Camera sensor's configuration.
    class CameraConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Returns the intrinsic calibration matrix of the camera as:
        //!  [fx  0 cx]
        //!  [ 0 fy cy]
        //!  [ 0  0  1]
        //! where:
        //!  - fx, fy : the focal lengths in meters
        //!  - cx, cy : principal point in pixels.
        virtual AZ::Matrix3x3 GetCameraMatrix() const = 0;

        /// Gets the vertical field of view in degrees.
        virtual float GetVerticalFOV() const = 0;
        /// Sets the vertical field of view in degrees.
        virtual void SetVerticalFOV(float value) = 0;

        /// Gets the camera image width in pixels.
        virtual int GetWidth() const = 0;
        /// Sets the camera image width in pixels.
        virtual void SetWidth(int value) = 0;

        /// Gets the camera image height in pixels.
        virtual int GetHeight() const = 0;
        /// Sets the camera image height in pixels.
        virtual void SetHeight(int value) = 0;

        /// Checks if the color camera is enabled.
        virtual bool IsColorCamera() const = 0;
        /// Enables or disables the color camera.
        virtual void SetColorCamera(bool value) = 0;

        /// Checks if the depth camera is enabled.
        virtual bool IsDepthCamera() const = 0;
        /// Enables or disables the depth camera.
        virtual void SetDepthCamera(bool value) = 0;

        /// Gets the near clip distance of the camera.
        virtual float GetNearClipDistance() const = 0;
        /// Sets the near clip distance of the camera.
        virtual void SetNearClipDistance(float value) = 0;

        /// Gets the far clip distance of the camera.
        virtual float GetFarClipDistance() const = 0;
        /// Sets the far clip distance of the camera.
        virtual void SetFarClipDistance(float value) = 0;
    };

    using CameraConfigurationRequestBus = AZ::EBus<CameraConfigurationRequest>;
} // namespace ROS2Sensors
