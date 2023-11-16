/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Math/Vector3.h>

namespace ROS2::WGS
{

    //! WGS84Coordinate is a 3D vector with double precision.
    //! It is used to represent coordinates in WSG84 coordinate system.
    struct WGS84Coordinate
    {
        AZ_RTTI(WGS84Coordinate, "{577a5637-b31a-44c5-a33f-50df2922af2a}");
        static void Reflect(AZ::ReflectContext* context);

        WGS84Coordinate();
        virtual ~WGS84Coordinate() = default;

        WGS84Coordinate(double latitude, double longitude, double altitude);
        explicit WGS84Coordinate(const AZ::Vector3& latLonAlt);

        [[nodiscard]] AZ::Vector3 ToVector3f() const;

        double m_latitude = 0.0; //!< Latitude in degrees.
        double m_longitude = 0.0; //!< Longitude in degrees.
        double m_altitude = 0.0; //!< Altitude in meters.
    };

    //! Vector3d is a 3D vector with double precision.
    //! It is used to represent coordinates in ECEF or ENU coordinate systems.
    struct Vector3d{
        Vector3d() = default;
        Vector3d(double x, double y, double z);
        explicit Vector3d(const AZ::Vector3& xyz);
        [[nodiscard]] AZ::Vector3 ToVector3f() const;

        Vector3d operator+(Vector3d const& v) const;
        Vector3d operator-(Vector3d const& v) const;

        double m_x = 0.0; //!< X coordinate in meters.
        double m_y = 0.0; //! Y coordinate in meters.
        double m_z = 0.0; //! Z coordinate in meters.
    };
}