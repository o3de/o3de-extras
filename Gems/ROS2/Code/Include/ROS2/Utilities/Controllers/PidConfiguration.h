/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Serialization/SerializeContext.h>
#include <control_toolbox/pid.hpp>

namespace ROS2::Controllers
{
    //! A wrapper for ROS 2 control_toolbox pid controller.
    //! @see <a href="https://github.com/ros-controls/control_toolbox">control_toolbox</a>.
    class PidConfiguration
    {
    public:
        AZ_TYPE_INFO(PidConfiguration, "{814E0D1E-2C33-44A5-868E-C914640E2F7E}");
        static void Reflect(AZ::ReflectContext* context);
        PidConfiguration() = default;

        //! Parametrized constructor of PidConfiguration
        //! @param p Proportional gain.
        //! @param i Integral gain.
        //! @param d Derivative gain.
        //! @param iMax Maximal allowable integral term.
        //! @param iMin Minimal allowable integral term.
        //! @param antiWindup Prevents condition of integrator overflow in integral action.
        //! @param outputLimit Limit PID output; set to 0.0 to disable.
        PidConfiguration(
            const double p,
            const double i,
            const double d,
            const double iMax,
            const double iMin,
            const bool antiWindup,
            const double outputLimit);

        //! Initialize PID using member fields as set by the user.
        void InitializePid();

        //! Compute the value of PID command.
        //! @param error Value of difference between target and state since last call.
        //! @param deltaTimeNanoseconds change in time since last call (nanoseconds).
        //! @returns Value of computed command.
        double ComputeCommand(double error, uint64_t deltaTimeNanoseconds);

    private:
        double m_p = 1.0; //!< proportional gain.
        double m_i = 0.0; //!< integral gain.
        double m_d = 0.0; //!< derivative gain.
        double m_iMax = 10.0; //!< maximal allowable integral term.
        double m_iMin = -10.0; //!< minimal allowable integral term.
        bool m_antiWindup = false; //!< prevents condition of integrator overflow in integral action.
        double m_outputLimit = 0.0; //!< limit PID output; set to 0.0 to disable.

        control_toolbox::Pid m_pid; //!< PID implementation object from control_toolbox (of ros2_control).
    };
} // namespace ROS2::Controllers
