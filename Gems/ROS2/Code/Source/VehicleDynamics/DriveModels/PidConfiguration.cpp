/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "VehicleDynamics/DriveModels/PidConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace VehicleDynamics
{
    void PidConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PidConfiguration>()
                ->Version(1)
                ->Field("P", &PidConfiguration::m_p)
                ->Field("I", &PidConfiguration::m_i)
                ->Field("D", &PidConfiguration::m_d)
                ->Field("IMin", &PidConfiguration::m_iMin)
                ->Field("IMax", &PidConfiguration::m_iMax)
                ->Field("Anti windup", &PidConfiguration::m_antiWindup);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<PidConfiguration>("PID configuration", "Configures a PID controller")
                    ->DataElement(1, &PidConfiguration::m_p, "P", "Proportional gain")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_i, "I", "Integral gain")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_d, "D", "Derivative gain")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_iMin, "IMin", "Minimum allowable integral term")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_iMax, "IMax", "Maximum allowable integral term")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_antiWindup, "AntiWindup", "Anti windup");
            }
        }
    }

    void PidConfiguration::InitializePid()
    {
        m_pid.initPid(m_p, m_i, m_d, m_iMax, m_iMin, m_antiWindup);
    }

    double PidConfiguration::ComputeCommand(double error, uint64_t deltaTimeNanoseconds)
    {
        return m_pid.computeCommand(error, deltaTimeNanoseconds);
    }
} // namespace VehicleDynamics
