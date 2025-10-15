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
#include <ROS2Sensors/Camera/CameraSensorConfiguration.h>

namespace ROS2Sensors
{
    //! Interface that allows to get and set Camera sensor's configuration.
    class CameraConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Returns the current configuration of the component.
        virtual const CameraSensorConfiguration GetConfiguration() = 0;

        //! Sets the configuration of the component.
        //! Each component should handle the configuration change without fully reinitializing the ROS2 publisher.
        //! This will allow to change the configuration of the component at runtime.
        //! Note: this method does not verify the configuration, so it is up to the caller to ensure that the configuration is valid.
        //! @param configuration The new configuration to set.
        virtual void SetConfiguration(const CameraSensorConfiguration& configuration) = 0;

        //! Returns the intrinsic calibration matrix of the camera as:
        //!  [fx  0 cx]
        //!  [ 0 fy cy]
        //!  [ 0  0  1]
        //! where:
        //!  - fx, fy : the focal lengths in meters
        //!  - cx, cy : principal point in pixels.
        virtual AZ::Matrix3x3 GetCameraMatrix() = 0;

        //! Gets the vertical field of view in degrees.
        virtual float GetVerticalFOV() = 0;
        //! Sets the vertical field of view in degrees.
        virtual void SetVerticalFOV(float value) = 0;

        //! Gets the camera image width in pixels.
        virtual int GetWidth() = 0;
        //! Sets the camera image width in pixels.
        virtual void SetWidth(int value) = 0;

        //! Gets the camera image height in pixels.
        virtual int GetHeight() = 0;
        //! Sets the camera image height in pixels.
        virtual void SetHeight(int value) = 0;

        //! Checks if the color camera is enabled.
        virtual bool IsColorCamera() = 0;
        //! Enables or disables the color camera.
        virtual void SetColorCamera(bool value) = 0;

        //! Checks if the depth camera is enabled.
        virtual bool IsDepthCamera() = 0;
        //! Enables or disables the depth camera.
        virtual void SetDepthCamera(bool value) = 0;

        //! Gets the near clip distance of the camera.
        virtual float GetNearClipDistance() = 0;
        //! Sets the near clip distance of the camera.
        virtual void SetNearClipDistance(float value) = 0;

        //! Gets the far clip distance of the camera.
        virtual float GetFarClipDistance() = 0;
        //! Sets the far clip distance of the camera.
        virtual void SetFarClipDistance(float value) = 0;
    };

    using CameraConfigurationRequestBus = AZ::EBus<CameraConfigurationRequest>;
} // namespace ROS2Sensors
