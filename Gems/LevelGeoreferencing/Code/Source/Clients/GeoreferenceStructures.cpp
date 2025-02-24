/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Georeferencing/GeoreferenceStructures.h>

namespace Georeferencing::WGS
{
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
        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<WGS84Coordinate>("WGS84Coordinate")
                ->Constructor<>()
                ->Constructor<double, double, double>()
                ->Attribute(AZ::Script::Attributes::Category, "Georeferencing")
                ->Method("ToVector3f", &WGS84Coordinate::ToVector3f)
                ->Method("FromVector3f", &WGS84Coordinate::FromVector3f)
                ->Method("SetLatitude", &WGS84Coordinate::SetLatitude)
                ->Method("SetLongitude", &WGS84Coordinate::SetLongitude)
                ->Method("SetAltitude", &WGS84Coordinate::SetAltitude)
                ->Method("GetLatitude", &WGS84Coordinate::GetLatitude)
                ->Method("GetLongitude", &WGS84Coordinate::GetLongitude)
                ->Method("GetAltitude", &WGS84Coordinate::GetAltitude);
        }
    }

} // namespace Georeferencing::WGS
