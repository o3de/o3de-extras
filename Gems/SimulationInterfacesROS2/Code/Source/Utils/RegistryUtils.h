/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/string/string.h>

namespace SimulationInterfacesROS2::RegistryUtilities
{
    // prefix for settings registry related to ros2 topics names
    inline constexpr const char* RegistryPrefix = "/SimulationInterfacesROS2";

    //! Gets name of the service with defined type form settings registry
    //! @return string with service name, if setting registry doesn't exits returns empty string
    [[nodiscard]] AZStd::string GetServiceName(AZStd::string serviceType);
} // namespace SimulationInterfacesROS2::RegistryUtilities
