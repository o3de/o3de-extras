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
                ->Field(GnssSensorConstants::GpsFixConfigName, &GnssSensorConfiguration::m_useGpsFix)
                ->Field(GnssSensorConstants::QuaternionConfigName, &GnssSensorConfiguration::m_useQuaternion)
                ->Field(GnssSensorConstants::NavSatConfigName, &GnssSensorConfiguration::m_useNavSat);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<GnssSensorConfiguration>("GNSS Sensor Configuration", "Configure one or more Topic-Types that may contain orientation")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GnssSensorConfiguration::m_useGpsFix,
                                                                  GnssSensorConstants::GpsFixConfigName,
                                                                  GnssSensorConstants::GpsFixMessageType)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GnssSensorConfiguration::m_useQuaternion,
                                                                  GnssSensorConstants::QuaternionConfigName,
                                                                  GnssSensorConstants::QuaternionMessageType)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GnssSensorConfiguration::m_useNavSat,
                                                                  GnssSensorConstants::NavSatConfigName,
                                                                  GnssSensorConstants::NavSatMessageType);
            }
        }
    }
} // namespace ROS2Sensors
