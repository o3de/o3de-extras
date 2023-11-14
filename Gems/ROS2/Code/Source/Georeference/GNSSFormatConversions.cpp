/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Georeference/GNSSFormatConversions.h"

constexpr double earthSemimajorAxis = 6378137.0;
constexpr double reciprocalFlattening = 1.0 / 298.257223563;
constexpr double earthSemiminorAxis = earthSemimajorAxis * (1.0 - reciprocalFlattening);
constexpr double firstEccentricitySquared = 2.0 * reciprocalFlattening - reciprocalFlattening * reciprocalFlattening;
constexpr double secondEccentrictySquared =
    reciprocalFlattening * (2.0 - reciprocalFlattening) / ((1.0 - reciprocalFlattening) * (1.0 - reciprocalFlattening));

// Based on http://wiki.gis.com/wiki/index.php/Geodetic_system
namespace ROS2::Utils::GeodeticConversions
{
    inline double DegToRad(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    inline double RadToDeg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    WGS::Vector3d WGS84ToECEF(const WGS::WGS84Coordinate& latitudeLongitudeAltitude)
    {
        const double latitudeRad = DegToRad(latitudeLongitudeAltitude.m_latitude);
        const double longitudeRad = DegToRad(latitudeLongitudeAltitude.m_longitude);
        const double altitude = latitudeLongitudeAltitude.m_altitude;

        const double helper = std::sqrt(1.0f - firstEccentricitySquared * std::sin(latitudeRad) * std::sin(latitudeRad));

        const double X = (earthSemimajorAxis / helper + altitude) * std::cos(latitudeRad) * std::cos(longitudeRad);
        const double Y = (earthSemimajorAxis / helper + altitude) * std::cos(latitudeRad) * std::sin(longitudeRad);
        const double Z = (earthSemimajorAxis * (1.0 - firstEccentricitySquared) / helper + altitude) * std::sin(latitudeRad);

        return { X, Y, Z };
    }

    WGS::Vector3d ECEFToENU(const WGS::WGS84Coordinate& referenceLatitudeLongitudeAltitude, const WGS::Vector3d& ECEFPoint)
    {
        const WGS::Vector3d referencePointInECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);
        const WGS::Vector3d pointToReferencePointECEF = ECEFPoint - referencePointInECEF;

        const double referenceLatitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.m_latitude);
        const double referenceLongitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.m_longitude);

        return { -sin(referenceLongitudeRad) * pointToReferencePointECEF.m_x + cos(referenceLongitudeRad) * pointToReferencePointECEF.m_y,
                 -sin(referenceLatitudeRad) * cos(referenceLongitudeRad) * pointToReferencePointECEF.m_x -
                     sin(referenceLatitudeRad) * sin(referenceLongitudeRad) * pointToReferencePointECEF.m_y +
                     cos(referenceLatitudeRad) * pointToReferencePointECEF.m_z,
                 cos(referenceLatitudeRad) * cos(referenceLongitudeRad) * pointToReferencePointECEF.m_x +
                     cos(referenceLatitudeRad) * sin(referenceLongitudeRad) * pointToReferencePointECEF.m_y +
                     sin(referenceLatitudeRad) * pointToReferencePointECEF.m_z };
    }

    WGS::Vector3d ENUToECEF(const WGS::WGS84Coordinate& referenceLatitudeLongitudeAltitude, const WGS::Vector3d& ENUPoint)
    {
        const auto referenceECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);

        const double referenceLatitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.m_latitude);
        const double referenceLongitudeRad = DegToRad(referenceLatitudeLongitudeAltitude.m_longitude);
        const double& e = ENUPoint.m_x;
        const double& n = ENUPoint.m_y;
        const double& u = ENUPoint.m_z;

        return { -std::sin(referenceLongitudeRad) * e - std::cos(referenceLongitudeRad) * std::sin(referenceLatitudeRad) * n +
                     std::cos(referenceLongitudeRad) * std::cos(referenceLatitudeRad) * u + referenceECEF.m_x,
                 std::cos(referenceLongitudeRad) * e - std::sin(referenceLongitudeRad) * std::sin(referenceLatitudeRad) * n +
                     std::cos(referenceLatitudeRad) * std::sin(referenceLongitudeRad) * u + referenceECEF.m_y,
                 std::cos(referenceLatitudeRad) * n + std::sin(referenceLatitudeRad) * u + referenceECEF.m_z };
    }

    WGS::WGS84Coordinate ECEFToWGS84(const WGS::Vector3d& ECFEPoint)
    {
        const double& x = ECFEPoint.m_x;
        const double& y = ECFEPoint.m_y;
        const double& z = ECFEPoint.m_z;

        const double radiusSquared = x * x + y * y;
        const double radius = std::sqrt(radiusSquared);

        const double E2 = earthSemimajorAxis * earthSemimajorAxis - earthSemiminorAxis * earthSemiminorAxis;
        const double F = 54.0 * earthSemiminorAxis * earthSemiminorAxis * z * z;
        const double G = radiusSquared + (1.0 - firstEccentricitySquared) * z * z - firstEccentricitySquared * E2;
        const double c = (firstEccentricitySquared * firstEccentricitySquared * F * radiusSquared) / (G * G * G);
        const double s = std::pow(1. + c + std::sqrt(c * c + 2. * c), 1. / 3);
        const double P = F / (3.0 * (s + 1.0 / s + 1.0) * (s + 1.0 / s + 1.0) * G * G);
        const double Q = std::sqrt(1.0 + 2.0 * firstEccentricitySquared * firstEccentricitySquared * P);

        const double ro = -(firstEccentricitySquared * P * radius) / (1.0 + Q) +
            std::sqrt(
                (earthSemimajorAxis * earthSemimajorAxis / 2.0) * (1.0 + 1.0 / Q) -
                ((1.0 - firstEccentricitySquared) * P * z * z) / (Q * (1.0 + Q)) - P * radiusSquared / 2.0);
        const double tmp = (radius - firstEccentricitySquared * ro) * (radius - firstEccentricitySquared * ro);
        const double U = std::sqrt(tmp + z * z);
        const double V = std::sqrt(tmp + (1.0 - firstEccentricitySquared) * z * z);
        const double zo = (earthSemiminorAxis * earthSemiminorAxis * z) / (earthSemimajorAxis * V);

        const double latitude = std::atan((z + secondEccentrictySquared * zo) / radius);
        const double longitude = std::atan2(y, x);
        const double altitude = U * (1.0 - earthSemiminorAxis * earthSemiminorAxis / (earthSemimajorAxis * V));

        return { RadToDeg(latitude), RadToDeg(longitude), altitude };
    }

} // namespace ROS2::GNSS
