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
    class ImuNoiseRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Returns the width of the camera sensor in pixels
        virtual void ConfigureVariance(
            [[maybe_unused]] const AZ::Vector3& accelerationVariance,
            [[maybe_unused]] const AZ::Vector3& angularVelocityVariance,
            [[maybe_unused]] const AZ::Vector3& orientationVariance)
        {
        }

        virtual void ApplyAccelerationNoise([[maybe_unused]] AZ::Vector3& linearAcceleration)
        {
        }
        virtual void ApplyAngularVelocityNoise([[maybe_unused]] AZ::Vector3& angularVelocity)
        {
        }
        virtual void ApplyOrientationNoise([[maybe_unused]] AZ::Quaternion& orientation)
        {
        }
    };

    using ImuNoiseRequestBus = AZ::EBus<ImuNoiseRequest>;
} // namespace ROS2
