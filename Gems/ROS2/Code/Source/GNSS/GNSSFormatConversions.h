/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "AzCore/Math/Matrix4x4.h"

namespace ROS2::GNSS {
    //! Converts radians to degrees
    float Rad2Deg(float rad);

    //! Converts degrees to radians
    float Deg2Rad(float deg);

    //! Converts point in 1984 World Geodetic System (GS84) to Earth Centred Earth Fixed (ECEF)
    //! @param latitudeLongitudeAltitude - point's latitude, longitude and altitude as 3d vector.
    //!     latitude and longitude are in decimal degrees
    //!     altitude is in meters
    //! @return 3d vector of ECEF coordinates.
    AZ::Vector3 WGS84ToECEF(const AZ::Vector3& latitudeLongitudeAltitude);

    //! Converts Earth Centred Earth Fixed (ECEF) coordinates to local east, north, up (ENU)
    //! @param referenceLatitudeLongitudeAltitude - reference point's latitude, longitude and altitude as 3d vector.
    //!     latitude and longitude are in decimal degrees
    //!     altitude is in meters
    //! @param ECEFPoint - ECEF point to bo converted.
    //! @return 3d vector of local east, north, up (ENU) coordinates.
    AZ::Vector3 ECEFToENU(const AZ::Vector3& referenceLatitudeLongitudeAltitude, const AZ::Vector3& ECEFPoint);

    //! Converts local east, north, up (ENU) coordinates to Earth Centred Earth Fixed (ECEF)
    //! @param referenceLatitudeLongitudeAltitude - reference point's latitude, longitude and altitude as 3d vector.
    //!     latitude and longitude are in decimal degrees
    //!     altitude is in meters
    //! @param ENUPoint - ENU point to bo converted.
    //! @return 3d vector of ECEF coordinates.
    AZ::Vector3 ENUToECEF(const AZ::Vector3& referenceLatitudeLongitudeAltitude, const AZ::Vector3& ENUPoint);

    //! Converts point in Earth Centred Earth Fixed (ECEF) to  984 World Geodetic System (GS84)
    //! @param ECEFPoint - ECEF point to bo converted.
    //! @return point's latitude, longitude and altitude as 3d vector.
    //!     latitude and longitude are in decimal degrees
    //!     altitude is in meters
    AZ::Vector3 ECEFToWGS84(const AZ::Vector3& ECFEPoint);
} // namespace ROS2::GNSS
