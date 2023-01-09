/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "AckermannModelLimits.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
   void AckermannModelLimits::Reflect(AZ::ReflectContext* context)
   {
       if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
       {
           serialize->Class<AckermannModelLimits>()
               ->Version(1)
               ->Field("SpeedLimit", &AckermannModelLimits::m_speedLimit)
               ->Field("SteeringLimit", &AckermannModelLimits::m_steeringLimit);


           if (AZ::EditContext* ec = serialize->GetEditContext())
           {
               ec->Class<AckermannModelLimits>("Skid steering Model Limits", "Limitations of speed, steering angles and other values")
                   ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                   ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &AckermannModelLimits::m_speedLimit, "Speed Limit", "Max linear speed (mps)")
                   ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                   ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                   ->DataElement(
                       AZ::Edit::UIHandlers::Default, &AckermannModelLimits::m_steeringLimit, "Steering Limit", "Max steering angle (rad)")
                   ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                   ->Attribute(AZ::Edit::Attributes::Max, 1.57f);
           }
       }
   }

   VehicleInputsState AckermannModelLimits::LimitState(const VehicleInputsState& inputState) const
   {
       VehicleInputsState ret = inputState;
       ret.m_angularRates = AZ::Vector3{0};
       ret.m_speed = AZ::Vector3{ LimitValue(ret.m_speed.GetX(), m_speedLimit),0.f,0.f};
       if (!ret.m_jointConfiguration.empty()){
           ret.m_jointConfiguration.front()= LimitValue(ret.m_jointConfiguration.front(), m_steeringLimit );
       }
       return ret;
   }

   VehicleInputsState AckermannModelLimits::GetMaximumState() const{
       VehicleInputsState ret;
       ret.m_speed = {m_speedLimit,0,0};
       ret.m_jointConfiguration = {m_steeringLimit};
       return ret;
   }
} // namespace ROS2::VehicleDynamics
