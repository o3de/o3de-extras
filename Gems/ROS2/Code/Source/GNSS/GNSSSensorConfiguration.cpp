/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GNSSSensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void GNSSSensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GNSSSensorConfiguration>()
                ->Version(1)
                ->Field("originLatitude", &GNSSSensorConfiguration::m_originLatitudeDeg)
                ->Field("originLongitude", &GNSSSensorConfiguration::m_originLongitudeDeg)
                ->Field("originAltitude", &GNSSSensorConfiguration::m_originAltitude);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<GNSSSensorConfiguration>("ROS2 GNSS Sensor", "GNSS sensor component")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GNSSSensorConfiguration::m_originLatitudeDeg,
                        "Latitude offset",
                        "GNSS latitude position offset in degrees")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GNSSSensorConfiguration::m_originLongitudeDeg,
                        "Longitude offset",
                        "GNSS longitude position offset in degrees")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GNSSSensorConfiguration::m_originAltitude,
                        "Altitude offset",
                        "GNSS altitude position offset in meters");
            }
        }
    }

} // namespace ROS2
