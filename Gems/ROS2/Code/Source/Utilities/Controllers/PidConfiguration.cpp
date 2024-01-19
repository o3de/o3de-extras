/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>

namespace ROS2::Controllers
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
                ->Field("Anti windup", &PidConfiguration::m_antiWindup)
                ->Field("Output limit", &PidConfiguration::m_outputLimit);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<PidConfiguration>("PID configuration", "Configures a PID controller")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_p, "P", "Proportional gain")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_i, "I", "Integral gain")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_d, "D", "Derivative gain")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_iMin, "IMin", "Minimum allowable integral term")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_iMax, "IMax", "Maximum allowable integral term")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidConfiguration::m_antiWindup, "AntiWindup", "Anti windup")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PidConfiguration::m_outputLimit,
                        "OutputLimit",
                        "Limit of the PID output [0, INF]. Set to 0.0 to disable.")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0);
            }
        }
    }
    PidConfiguration::PidConfiguration(
        const double p,
        const double i,
        const double d,
        const double iMax,
        const double iMin,
        const bool antiWindup,
        const double outputLimit)
        : m_p(p)
        , m_i(i)
        , m_d(d)
        , m_iMax(iMax)
        , m_iMin(iMin)
        , m_antiWindup(antiWindup)
        , m_outputLimit(outputLimit)
    {
    }

    void PidConfiguration::InitializePid()
    {
        m_pid.initPid(m_p, m_i, m_d, m_iMax, m_iMin, m_antiWindup);
    }

    double PidConfiguration::ComputeCommand(double error, uint64_t deltaTimeNanoseconds)
    {
        double output = m_pid.computeCommand(error, deltaTimeNanoseconds);
        if (m_outputLimit > 0.0)
        {
            output = AZStd::clamp<float>(output, -m_outputLimit, m_outputLimit);
        }
        return output;
    }
} // namespace ROS2::Controllers
