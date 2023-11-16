/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Vector3.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Georeference/GeoreferenceStructures.h>

namespace ROS2::WGS
{

    WGS84Coordinate::WGS84Coordinate()
        : m_latitude(0.0)
        , m_longitude(0.0)
        , m_altitude(0.0)
    {
    }

    WGS84Coordinate::WGS84Coordinate(double latitude, double longitude, double altitude)
        : m_latitude(latitude)
        , m_longitude(longitude)
        , m_altitude(altitude)
    {
    }

    WGS84Coordinate::WGS84Coordinate(const AZ::Vector3& latLonAlt)
        : m_latitude(latLonAlt.GetX())
        , m_longitude(latLonAlt.GetY())
        , m_altitude(latLonAlt.GetZ())
    {
    }

    void WGS84Coordinate::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WGS84Coordinate>()
                ->Version(1)
                ->Field("Altitude", &WGS84Coordinate::m_altitude)
                ->Field("Latitude", &WGS84Coordinate::m_latitude)
                ->Field("Longitude", &WGS84Coordinate::m_longitude);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<WGS84Coordinate>("WGS84 Coordinate", "WGS84 coordinate")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WGS84Coordinate::m_latitude,
                        "Latitude",
                        "Latitude in degrees, WGS84 ellipsoid, west is negative")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WGS84Coordinate::m_longitude,
                        "Longitude",
                        "Longitude in degrees, WGS84 ellipsoid, south is negative")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &WGS84Coordinate::m_altitude, "Altitude", "Altitude in meters, WGS84 ellipsoid");
            }
        }
    }

    AZ::Vector3 WGS84Coordinate::ToVector3f() const
    {
        return AZ::Vector3(static_cast<float>(m_latitude), static_cast<float>(m_longitude), static_cast<float>(m_altitude));
    }

    Vector3d::Vector3d(double x, double y, double z)
        : m_x(x)
        , m_y(y)
        , m_z(z)
    {
    }

    [[maybe_unused]] Vector3d::Vector3d(const AZ::Vector3& xyz)
        : m_x(xyz.GetX())
        , m_y(xyz.GetY())
        , m_z(xyz.GetZ())
    {
    }

    AZ::Vector3 Vector3d::ToVector3f() const
    {
        return AZ::Vector3(static_cast<float>(m_x), static_cast<float>(m_y), static_cast<float>(m_z));
    }

    Vector3d Vector3d::operator+(Vector3d const& v) const
    {
        Vector3d r;
        r.m_x = m_x + v.m_x;
        r.m_y = m_y + v.m_y;
        r.m_z = m_z + v.m_z;
        return r;
    }

    Vector3d Vector3d::operator-(Vector3d const& v) const
    {
        Vector3d r;
        r.m_x = m_x - v.m_x;
        r.m_y = m_y - v.m_y;
        r.m_z = m_z - v.m_z;
        return r;
    }

} // namespace ROS2::WGS
