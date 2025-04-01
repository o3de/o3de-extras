/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

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
} // namespace ROS2
