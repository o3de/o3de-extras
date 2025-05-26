/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "GeoreferencingTypeIds.h"
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>

namespace Georeferencing::WGS
{

    //! WGS84Coordinate is a 3D vector with double precision.
    //! It is used to represent coordinates in WGS84 coordinate system.
    struct WGS84Coordinate
    {
        AZ_RTTI(WGS84Coordinate, WGS84CoordinateTypeId);
        static void Reflect(AZ::ReflectContext* context);

        WGS84Coordinate() = default;
        virtual ~WGS84Coordinate() = default;

        WGS84Coordinate(double latitude, double longitude, double altitude)
            : m_latitude(latitude)
            , m_longitude(longitude)
            , m_altitude(altitude)
        {
        }

        explicit WGS84Coordinate(const AZ::Vector3& latLonAlt)
            : m_latitude(latLonAlt.GetX())
            , m_longitude(latLonAlt.GetY())
            , m_altitude(latLonAlt.GetZ())
        {
        }

        void FromVector3f(const AZ::Vector3& latLonAlt)
        {
            m_latitude = latLonAlt.GetX();
            m_longitude = latLonAlt.GetY();
            m_altitude = latLonAlt.GetZ();
        }

        //! Converts WGS84Coordinate to Vector3f, where x is latitude, y is longitude
        //! and z is altitude. Expect accuracy loss due to float conversion.
        AZ::Vector3 ToVector3f() const
        {
            return AZ::Vector3(static_cast<float>(m_latitude), static_cast<float>(m_longitude), static_cast<float>(m_altitude));
        }

        bool operator==(const WGS84Coordinate& rhs) const
        {
            return AZ::IsClose(m_latitude, rhs.m_latitude) && AZ::IsClose(m_longitude, rhs.m_longitude) &&
                AZ::IsClose(m_altitude, rhs.m_altitude);
        }

        void SetLatitude(double latitude)
        {
            m_latitude = latitude;
        }

        void SetLongitude(double longitude)
        {
            m_longitude = longitude;
        }

        void SetAltitude(double altitude)
        {
            m_altitude = altitude;
        }

        double GetLatitude() const
        {
            return m_latitude;
        }

        double GetLongitude() const
        {
            return m_longitude;
        }
        double GetAltitude() const
        {
            return m_altitude;
        }

        double m_latitude = 0.0; //!< Latitude in degrees.
        double m_longitude = 0.0; //!< Longitude in degrees.
        double m_altitude = 0.0; //!< Altitude in meters.
    };
} // namespace Georeferencing::WGS
