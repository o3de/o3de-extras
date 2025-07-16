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

namespace ROS2Controllers {
    void RigidBodyTwistControlComponentConfig::Reflect(AZ::ReflectContext *context) {
        if (AZ::SerializeContext *serialize = azrtti_cast<AZ::SerializeContext *>(context)) {
            serialize->Class<RigidBodyTwistControlComponentConfig>()
                    ->Version(1)
                    ->Field("PhysicalApi", &RigidBodyTwistControlComponentConfig::m_physicalApi)
                    ->Field("LinerControllers", &RigidBodyTwistControlComponentConfig::m_linerControllers)
                    ->Field("AngularControllers", &RigidBodyTwistControlComponentConfig::m_angularControllers);


            if (AZ::EditContext *ec = serialize->GetEditContext()) {
                ec->Class<RigidBodyTwistControlComponentConfig>("Rigid Body Twist Control Config",
                                                                "Configuration for Rigid Body Twist Control Component")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                        ->DataElement(AZ::Edit::UIHandlers::ComboBox,
                                      &RigidBodyTwistControlComponentConfig::m_physicalApi, "Physical API",
                                      "API to use for applying velocities. Velocity mode directly sets velocities (limited control). Force mode uses PID controllers for better control.")
                        ->EnumAttribute(PhysicalApi::Kinematic,
                                        "Kinematic - Directly sets the kinematic state of the rigid body")
                        ->EnumAttribute(PhysicalApi::Velocity, "Velocity - Direct velocity control")
                        ->EnumAttribute(PhysicalApi::Force, "Force - PID controlled force application per axis")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                        ->UIElement(AZ::Edit::UIHandlers::Label, "", "")
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, "")
                        ->Attribute(AZ::Edit::Attributes::ValueText, &RigidBodyTwistControlComponentConfig::GetNote)
                        ->DataElement(AZ::Edit::UIHandlers::Default,
                                      &RigidBodyTwistControlComponentConfig::m_linerControllers, "Linear Controllers",
                                      "PID controllers for linear velocities (only used in Force mode)")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                        ->Attribute(AZ::Edit::Attributes::Visibility,
                                    &RigidBodyTwistControlComponentConfig::GetPidControllersVisibility)
                        ->DataElement(AZ::Edit::UIHandlers::Default,
                                      &RigidBodyTwistControlComponentConfig::m_angularControllers,
                                      "Angular Controllers",
                                      "PID controllers for angular velocities (only used in Force mode)")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, false)
                        ->Attribute(AZ::Edit::Attributes::Visibility,
                                    &RigidBodyTwistControlComponentConfig::GetPidControllersVisibility);
            }
        }
    }

    AZStd::string RigidBodyTwistControlComponentConfig::GetNote() const {
        if (m_physicalApi == PhysicalApi::Force) {
            return "User defined PID controllers for linear and angular velocities. "
                   "These controllers are used to apply forces to the rigid body in order to achieve the desired twist control.";
        }
        if (m_physicalApi == PhysicalApi::Velocity) {
            return "Direct velocity control, with collision response. "
                   "The rigid body will be set to the desired linear and angular velocities."
                   "This mode limits the usage of odometry and IMU components";
        }
        if (m_physicalApi == PhysicalApi::Kinematic) {
            return "Kinematic mode directly sets the kinematic state of the rigid body. "
                   "No collision response is applied, and the rigid body will not be affected by physics simulation.";

        }
        return "";
    }


} // namespace ROS2Controllers
