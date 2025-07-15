/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RigidBodyTwistControlComponentConfig.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2Controllers
{
    void RigidBodyTwistControlComponentConfig::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RigidBodyTwistControlComponentConfig>()
                ->Version(1)
                ->Field("MaxLinearVelocity", &RigidBodyTwistControlComponentConfig::m_maxLinearVelocity)
                ->Field("MaxAngularVelocity", &RigidBodyTwistControlComponentConfig::m_maxAngularVelocity)
                ->Field("EnableVelocityLimiting", &RigidBodyTwistControlComponentConfig::m_enableVelocityLimiting)
                ->Field("PhysicalApi", &RigidBodyTwistControlComponentConfig::m_physicalApi)

                ->Field("LinerControllers", &RigidBodyTwistControlComponentConfig::m_linerControllers)
                ->Field("AngularControllers", &RigidBodyTwistControlComponentConfig::m_angularControllers);


            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RigidBodyTwistControlComponentConfig>("Rigid Body Twist Control Config", "Configuration for Rigid Body Twist Control Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RigidBodyTwistControlComponentConfig::m_maxLinearVelocity, "Max Linear Velocity", "Maximum linear velocity in m/s")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 50.0f)
                        ->Attribute(AZ::Edit::Attributes::Step, 0.1f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RigidBodyTwistControlComponentConfig::m_maxAngularVelocity, "Max Angular Velocity", "Maximum angular velocity in rad/s")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 10.0f)
                        ->Attribute(AZ::Edit::Attributes::Step, 0.1f)
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &RigidBodyTwistControlComponentConfig::m_enableVelocityLimiting, "Enable Velocity Limiting", "Enable velocity limiting")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &RigidBodyTwistControlComponentConfig::m_physicalApi, "Physical API", "API to use for applying velocities")
                        ->EnumAttribute(PhysicalApi::Velocity, "Velocity")
                        ->EnumAttribute(PhysicalApi::Force, "Force")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RigidBodyTwistControlComponentConfig::m_linerControllers, "Linear Controllers", "PID controllers for linear velocities")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RigidBodyTwistControlComponentConfig::m_angularControllers, "Angular Controllers", "PID controllers for angular velocities")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }
} // namespace ROS2Controllers