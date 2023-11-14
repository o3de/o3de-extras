/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "GeoreferenceStructures.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>

namespace ROS2
{
    //! Interface that allows to convert between level and WSG84 coordinates.
    class GeoreferenceRequests
    {
    public:

        //! Function converts from Level's coordinate system to WSG84.
        //! @param xyz Vector3 in Level's coordinate system.
        //! @return Vector3 in WSG84 coordinate system as @class WGS::WGS84Coordinate.
        virtual WGS::WGS84Coordinate ConvertFromLevelToWSG84(const AZ::Vector3& xyz) = 0;

        //! Function converts from WSG84 coordinate system to Level's.
        //!  @param latLon Vector3 in WSG84 coordinate system, where x is latitude, y is longitude and z is altitude.
        //!  @return Vector3 in Level's coordinate system.
        virtual AZ::Vector3 ConvertFromWSG84ToLevel(const WGS::WGS84Coordinate& latLon) = 0;

        //! Function returns rotation from Level's frame to ENU's (East-North-Up) rotation.
        //! Function is useful to fin georeference rotation of the level.
        //! @return Quaternion in ENU coordinate system.
        virtual AZ::Quaternion GetRotationFromLevelToENU() = 0;
    };

    class GeoreferenceRequestsTraits : public AZ::EBusTraits
    {
    public:
        // EBusTraits overrides ...
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using GeoreferenceRequestsBus = AZ::EBus<GeoreferenceRequests, GeoreferenceRequestsTraits>;
} // namespace ROS2
