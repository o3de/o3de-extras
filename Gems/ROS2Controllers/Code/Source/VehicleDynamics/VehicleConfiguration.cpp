/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleConfiguration.h"
#include "Utilities.h"
#include <AzCore/Debug/Trace.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void VehicleConfiguration::Reflect(AZ::ReflectContext* context)
    {
        AxleConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<VehicleConfiguration>()
                ->Version(1)
                ->Field("AxlesConfigurations", &VehicleConfiguration::m_axles)
                ->Field("Track", &VehicleConfiguration::m_track)
                ->Field("Wheelbase", &VehicleConfiguration::m_wheelbase);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<VehicleConfiguration>("Vehicle configuration", "Configuration of vehicle")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &VehicleConfiguration::m_axles, "Axles", "Configurations of axles for this vehicle")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleConfiguration::m_track,
                        "Track",
                        "The distance in meters between the hub flanges on an axle")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &VehicleConfiguration::m_wheelbase,
                        "Wheelbase",
                        "The distance in meters between the midpoints of vehicles front and rear wheels");
            }
        }
    }
} // namespace ROS2::VehicleDynamics
