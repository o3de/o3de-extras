/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointNamePositionPair.h"
#include <AzCore/Serialization/EditContext.h>

namespace ROS2
{
    void JointNamePositionPair::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointNamePositionPair>()
                ->Version(1)
                ->Field("Name", &JointNamePositionPair::m_name)
                ->Field("Position", &JointNamePositionPair::m_position);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<JointNamePositionPair>("JointNamePositionPair", "Pair of joint name and its position")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointNamePositionPair::m_name,
                        "Name",
                        "Joint name")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointNamePositionPair::m_position,
                        "Position",
                        "Initial joint position");
            }
        }
    }
} // namespace ROS2
