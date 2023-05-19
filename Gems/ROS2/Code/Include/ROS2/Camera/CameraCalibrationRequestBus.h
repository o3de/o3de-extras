/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Matrix4x4.h>

namespace ROS2
{
    //! Interface allows to obtain intrinsic parameters of the camera. To obtain extrinsic parameters use TransformProviderRequestBus.
    class CameraCalibrationRequest : public AZ::EBusTraits
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

        //! Returns the width of the camera sensor in pixels
        virtual int GetWidth() const = 0;

        //! Returns the height of the camera sensor in pixels
        virtual int GetHeight() const = 0;

        //! Returns the vertical field of view of the camera in degrees
        virtual float GetVerticalFOV() const = 0;
    };

    using CameraCalibrationRequestBus = AZ::EBus<CameraCalibrationRequest>;
} // namespace ROS2
