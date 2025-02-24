/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "GeoreferenceStructures.h"
#include "GeoreferencingTypeIds.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Vector3.h>

namespace Georeferencing
{

    //! Interface that allows to configure georeferencing of the level.
    class GeoreferenceConfigurationRequests
    {
    public:
        AZ_RTTI(GeoreferenceConfigurationRequests, GeoreferenceConfigurationRequestsTypeId);

        //! Function sets entity that represents the origin of the georeferencing.
        virtual void SetOriginEntity(const AZ::EntityId& entityId) = 0;

        //! Function sets location of the origin in WGS84 coordinate system.
        virtual void SetOriginCoordinates(const WGS::WGS84Coordinate& origin) = 0;

        //! Function returns entity that represents the origin of the georeferencing.
        virtual AZ::EntityId GetOriginEntity() = 0;

        //! Function returns location of the origin in WGS84 coordinate system.
        virtual WGS::WGS84Coordinate GetOriginCoordinates() = 0;
    };

    class GeoreferenceConfigurationRequestsTraits : public AZ::EBusTraits
    {
    public:
        // EBusTraits overrides ...
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using GeoreferenceConfigurationRequestsBus = AZ::EBus<GeoreferenceConfigurationRequests, GeoreferenceConfigurationRequestsTraits>;

    //! Interface that allows to convert between level and WGS84 coordinates.
    class GeoreferenceRequests
    {
    public:
        AZ_RTTI(GeoreferenceRequests, GeoreferenceRequestsTypeId);

        //! Function converts from Level's coordinate system to WGS84.
        //! @param xyz Vector3 in Level's coordinate system.
        //! @return Vector3 in WGS84 coordinate system as @class WGS::WGS84Coordinate.
        virtual WGS::WGS84Coordinate ConvertFromLevelToWGS84(const AZ::Vector3& xyz) = 0;

        //! Function converts from WGS84 coordinate system to Level's.
        //!  @param latLon Vector3 in WGS84 coordinate system, where x is latitude, y is longitude and z is altitude.
        //!  @return Vector3 in Level's coordinate system.
        virtual AZ::Vector3 ConvertFromWGS84ToLevel(const WGS::WGS84Coordinate& latLon) = 0;

        //! Function returns rotation from Level's frame to ENU's (East-North-Up) rotation.
        //! Function is useful to find georeferencing rotation of the level.
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
} // namespace Georeferencing
