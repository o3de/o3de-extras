/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/Math/Matrix4x4.h>
#include <ROS2/Georeference/GeoreferenceStructures.h>

namespace ROS2::Utils::GeodeticConversions
{

    //! Converts point in 1984 World Geodetic System (GS84) to Earth Centred Earth Fixed (ECEF)
    //! @param latitudeLongitudeAltitude - point's latitude, longitude and altitude as WGS::WGS84Coordinate.
    //!     latitude and longitude are in decimal degrees
    //!     altitude is in meters
    //! @return 3d vector of ECEF coordinates.
    WGS::Vector3d WGS84ToECEF(const WGS::WGS84Coordinate& latitudeLongitudeAltitude);

    //! Converts Earth Centred Earth Fixed (ECEF) coordinates to local east, north, up (ENU)
    //! @param referenceLatitudeLongitudeAltitude - reference point's latitude, longitude and altitude as 3d vector.
    //!     latitude and longitude are in decimal degrees
    //!     altitude is in meters
    //! @param ECEFPoint - ECEF point to bo converted.
    //! @return 3d vector of local east, north, up (ENU) coordinates.
    WGS::Vector3d ECEFToENU(const WGS::WGS84Coordinate& referenceLatitudeLongitudeAltitude, const WGS::Vector3d& ECEFPoint);

    //! Converts local east, north, up (ENU) coordinates to Earth Centred Earth Fixed (ECEF)
    //! @param referenceLatitudeLongitudeAltitude - reference point's latitude, longitude and altitude as WGS::WGS84Coordinate.
    //! @param ENUPoint - ENU point to bo converted.
    //! @return 3d vector of ECEF coordinates.
    WGS::Vector3d ENUToECEF(const WGS::WGS84Coordinate& referenceLatitudeLongitudeAltitude, const WGS::Vector3d& ENUPoint);

    //! Converts point in Earth Centred Earth Fixed (ECEF) to  984 World Geodetic System (GS84)
    //! @param ECEFPoint - ECEF point to bo converted.
    //! @return point's latitude, longitude and altitude as 3d vector.
    //!     latitude and longitude are in decimal degrees
    //!     altitude is in meters
    WGS::WGS84Coordinate ECEFToWGS84(const WGS::Vector3d& ECFEPoint);
} // namespace ROS2::Utils::GeodeticConversions
