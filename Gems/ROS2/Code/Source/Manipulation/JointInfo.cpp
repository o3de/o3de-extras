/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AzCore/Serialization/EditContext.h"
#include <ROS2/Manipulation/JointInfo.h>

namespace ROS2
{
    void JointInfo::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointInfo>()
                ->Version(1)
                ->Field("IsArticulation", &JointInfo::m_isArticulation)
                ->Field("Axis", &JointInfo::m_axis)
                ->Field("EntityComponentIdPair", &JointInfo::m_entityComponentIdPair)
                ->Field("RestPosition", &JointInfo::m_restPosition);
        }
    }

    void JointInitialPosition::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointInitialPosition>()
                ->Version(1)
                ->Field("Name", &JointInitialPosition::m_name)
                ->Field("Position", &JointInitialPosition::m_position)
                ->Field("Index", &JointInitialPosition::m_index);



        if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<JointInitialPosition>("JointInitialPosition", "Initial joint position and index.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointInitialPosition::m_name,
                        "Name",
                        "Joint Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointInitialPosition::m_position,
                        "Position",
                        "Position")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointInitialPosition::m_index,
                        "Index",
                        "Index used by Position Controller.")
                    ;
            }
        }


    }
} // namespace ROS2
