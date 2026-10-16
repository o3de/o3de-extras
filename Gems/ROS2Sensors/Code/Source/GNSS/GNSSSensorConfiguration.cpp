/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2Sensors/GNSS/GNSSSensorConfiguration.h>

namespace ROS2Sensors
{
    void GnssSensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GnssSensorConfiguration>()
                ->Version(1)
                ->Field(GnssSensorConstants::QuaternionConfigName, &GnssSensorConfiguration::m_useQuaternion)
                ->Field(GnssSensorConstants::GpsFixConfigName, &GnssSensorConfiguration::m_useGpsFix)
                ->Field(GnssSensorConstants::NavSatConfigName, &GnssSensorConfiguration::m_useNavSat);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<GnssSensorConfiguration>("GNSS Sensor Configuration", "Configure one or more Topic-Types that contain GNSS positiona and/or orientation")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GnssSensorConfiguration::m_useQuaternion,
                                                                  "Publish Orientation",
                                                                  GnssSensorConstants::QuaternionMessageType)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GnssSensorConfiguration::m_useGpsFix,
                                                                  "Publish GPSFix",
                                                                  GnssSensorConstants::GpsFixMessageType)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GnssSensorConfiguration::m_useNavSat,
                                                                  "Publish NavSatFix",
                                                                  GnssSensorConstants::NavSatMessageType);
            }
        }
    }
} // namespace ROS2Sensors
