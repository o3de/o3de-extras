/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GNSSFormatConversions.h"

constexpr double earthSemimajorAxis = 6378137.0f;
constexpr double reciprocalFlattening = 1 / 298.257223563f;
constexpr double earthSemiminorAxis = earthSemimajorAxis * (1.0f - reciprocalFlattening);
constexpr double firstEccentricitySquared = 2 * reciprocalFlattening - reciprocalFlattening * reciprocalFlattening;
constexpr double secondEccentrictySquared = reciprocalFlattening * (2.0f - reciprocalFlattening) / ((1.0f - reciprocalFlattening) * (1.0f - reciprocalFlattening));

// Based on http://wiki.gis.com/wiki/index.php/Geodetic_system
namespace ROS2::GNSS {
    float Rad2Deg(float rad) {
        return rad * 180.0f / AZ::Constants::Pi;
    }

    float Deg2Rad(float deg) {
        return deg * AZ::Constants::Pi / 180.0f;
    }

    AZ::Vector3 WGS84ToECEF(const AZ::Vector3& latitudeLongitudeAltitude) {
        const double latitudeRad = Deg2Rad(latitudeLongitudeAltitude.GetX());
        const double longitudeRad = Deg2Rad(latitudeLongitudeAltitude.GetY());
        const double altitude = latitudeLongitudeAltitude.GetZ();

        const double helper = AZStd::sqrt(1.0f - firstEccentricitySquared * AZStd::sin(latitudeRad) * AZStd::sin(latitudeRad));

        const double X = (earthSemimajorAxis / helper + altitude) * AZStd::cos(latitudeRad) * AZStd::cos(longitudeRad);
        const double Y = (earthSemimajorAxis / helper + altitude) * AZStd::cos(latitudeRad) * AZStd::sin(longitudeRad);
        const double Z = (earthSemimajorAxis * (1.0f - firstEccentricitySquared) / helper + altitude) * AZStd::sin(latitudeRad);

        return {static_cast<float>(X), static_cast<float>(Y), static_cast<float>(Z)};
    }

    AZ::Vector3 ECEFToENU(const AZ::Vector3& referenceLatitudeLongitudeAltitude, const AZ::Vector3& ECEFPoint) {
        AZ::Vector3 referencePointInECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);
        AZ::Vector3 pointToReferencePointECEF = ECEFPoint - referencePointInECEF;

        float referenceLatitudeRad = Deg2Rad(referenceLatitudeLongitudeAltitude.GetX());
        float referenceLongitudeRad = Deg2Rad(referenceLatitudeLongitudeAltitude.GetY());

        return {
                -AZStd::sin(referenceLongitudeRad) * pointToReferencePointECEF.GetX() + AZStd::cos(referenceLongitudeRad) * pointToReferencePointECEF.GetY(),
                -AZStd::sin(referenceLatitudeRad) * AZStd::cos(referenceLongitudeRad) * pointToReferencePointECEF.GetX() - AZStd::sin(referenceLatitudeRad) * AZStd::sin(referenceLongitudeRad) * pointToReferencePointECEF.GetY() + AZStd::cos(referenceLatitudeRad) * pointToReferencePointECEF.GetZ(),
                AZStd::cos(referenceLatitudeRad) * AZStd::cos(referenceLongitudeRad) * pointToReferencePointECEF.GetX() + AZStd::cos(referenceLatitudeRad) * AZStd::sin(referenceLongitudeRad) * pointToReferencePointECEF.GetY() + AZStd::sin(referenceLatitudeRad) * pointToReferencePointECEF.GetZ(),
        };
    }

    AZ::Vector3 ENUToECEF(const AZ::Vector3& referenceLatitudeLongitudeAltitude, const AZ::Vector3& ENUPoint) {
        AZ::Vector3 referenceECEF = WGS84ToECEF(referenceLatitudeLongitudeAltitude);

        const float referenceLatitudeRad = Deg2Rad(referenceLatitudeLongitudeAltitude.GetX());
        const float referenceLongitudeRad = Deg2Rad(referenceLatitudeLongitudeAltitude.GetY());
        const float e = ENUPoint.GetX();
        const float n = ENUPoint.GetY();
        const float u = ENUPoint.GetZ();

        return {-AZStd::sin(referenceLongitudeRad) * e - AZStd::cos(referenceLongitudeRad) * AZStd::sin(referenceLatitudeRad) * n + AZStd::cos(referenceLongitudeRad) * AZStd::cos(referenceLatitudeRad) * u + referenceECEF.GetX(),
                 AZStd::cos(referenceLongitudeRad) * e - AZStd::sin(referenceLongitudeRad) * AZStd::sin(referenceLatitudeRad) * n + AZStd::cos(referenceLatitudeRad) * AZStd::sin(referenceLongitudeRad) * u + referenceECEF.GetY(),
                 AZStd::cos(referenceLatitudeRad) * n + AZStd::sin(referenceLatitudeRad) * u + referenceECEF.GetZ()};
    }

    AZ::Vector3 ECEFToWGS84(const AZ::Vector3& ECFEPoint) {
        const double x = ECFEPoint.GetX();
        const double y = ECFEPoint.GetY();
        const double z = ECFEPoint.GetZ();

        const double radiusSquared = x * x + y * y;
        const double radius = AZStd::sqrt(radiusSquared);

        const double E2 = earthSemimajorAxis * earthSemimajorAxis - earthSemiminorAxis * earthSemiminorAxis;
        const double F = 54.0f * earthSemiminorAxis * earthSemiminorAxis * z * z;
        const double G = radiusSquared + (1.0f - firstEccentricitySquared) * z * z - firstEccentricitySquared * E2;
        const double c = (firstEccentricitySquared * firstEccentricitySquared * F * radiusSquared) / (G * G * G);
        const double s = std::cbrt(1.0f + c + AZStd::sqrt(c * c + 2.0f * c));
        const double P = F / (3.0f * (s + 1.0f / s + 1.0f) * (s + 1.0f / s + 1.0f) * G * G);
        const double Q = AZStd::sqrt(1.0f + 2.0f * firstEccentricitySquared * firstEccentricitySquared * P);

        const double ro = -(firstEccentricitySquared * P * radius) / (1.0f + Q) +
                AZStd::sqrt((earthSemimajorAxis * earthSemimajorAxis / 2.0f) * (1.0f + 1.0f / Q) -
                            ((1.0f - firstEccentricitySquared) * P * z * z) / (Q * (1.0f + Q)) - P * radiusSquared / 2.0f);
        const double tmp = (radius - firstEccentricitySquared * ro) * (radius - firstEccentricitySquared * ro);
        const double U = AZStd::sqrt(tmp + z * z);
        const double V = AZStd::sqrt(tmp + (1.0f - firstEccentricitySquared) * z * z);
        const double zo = (earthSemiminorAxis * earthSemiminorAxis * z) / (earthSemimajorAxis * V);

        const double latitude = AZStd::atan((z + secondEccentrictySquared * zo) / radius);
        const double longitude = AZStd::atan2(y, x);
        const double altitude = U * (1.0f - earthSemiminorAxis * earthSemiminorAxis / (earthSemimajorAxis * V));

        return {Rad2Deg(static_cast<float>(latitude)), Rad2Deg(static_cast<float>(longitude)), static_cast<float>(altitude)};
    }

} // namespace ROS2::GNSS
