/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Controllers/Controllers/PidConfiguration.h>

namespace ROS2Controllers::Controllers
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
        if (m_iMin > m_iMax)
        {
            AZ_Error("PidConfiguration", false, "Invalid PID configuration.");
        }
        else
        {
            m_initialized = true;
        }
    }

    double PidConfiguration::ComputeCommand(double error, uint64_t deltaTimeNanoseconds)
    {
        const double dt = aznumeric_cast<double>(deltaTimeNanoseconds) / 1.e9;

        if (!m_initialized)
        {
            AZ_Error("PidConfiguration", false, "PID not initialized, ignoring.");
            return 0.0;
        }

        if (dt <= 0.0 || !azisfinite(error))
        {
            AZ_Warning("PidConfiguration", false, "Invalid PID conditions.");
            return 0.0;
        }

        const double proportionalTerm = m_p * error;

        m_integral += error * dt;

        if (m_antiWindup && m_i != 0)
        {
            AZStd::pair<double, double> bounds = AZStd::minmax<double>(m_iMin / m_i, m_iMax / m_i);
            m_integral = AZStd::clamp<double>(m_integral, bounds.first, bounds.second);
        }

        double integralTerm = m_i * m_integral;

        if (m_antiWindup)
        {
            integralTerm = AZStd::clamp<double>(integralTerm, m_iMin, m_iMax);
        }

        const double derivative = (error - m_previousError) / dt;
        const double derivativeTerm = m_d * derivative;

        m_previousError = error;

        double output = proportionalTerm + integralTerm + derivativeTerm;

        if (m_outputLimit > 0.0)
        {
            output = AZStd::clamp<double>(output, 0.0, m_outputLimit);
        }
        return output;
    }
} // namespace ROS2Controllers::Controllers
