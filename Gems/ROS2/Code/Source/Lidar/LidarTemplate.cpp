/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarTemplate.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
   void LidarTemplate::Reflect(AZ::ReflectContext* context)
   {
       if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
       {   // TODO - some attributes are not reflected. Waiting for more realistic lidar templates
           serializeContext->Class<LidarTemplate>()
               ->Version(1)
               ->Field("Layers", &LidarTemplate::m_layers)
               ->Field("Points per layer", &LidarTemplate::m_numberOfIncrements)
               ->Field("Max range", &LidarTemplate::m_maxRange)
               ;

           if (AZ::EditContext* ec = serializeContext->GetEditContext())
           {
               ec->Class<LidarTemplate>("Lidar Template", "Lidar Template")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &LidarTemplate::m_layers, "Layers", "Vertical dimension")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                   ->DataElement(AZ::Edit::UIHandlers::Default, &LidarTemplate::m_numberOfIncrements, "Points per layer", "Horizontal dimension")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                   ->DataElement(AZ::Edit::UIHandlers::Default, &LidarTemplate::m_maxRange, "Max range", "Maximum beam range [m]")
                       ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                       ;
           }
       }
   }
} // namespace ROS2

