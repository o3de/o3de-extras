/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RegistryUtils.h"
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Settings/SettingsRegistry.h>

namespace ROS2SimulationInterfaces::RegistryUtilities
{
    AZStd::string GetName(AZStd::string serviceType)
    {
        AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
        AZ_Assert(settingsRegistry, "Settings Registry is not available");
        AZStd::string output = "";
        AZ::IO::Path setRegPath = AZ::IO::Path(RegistryPrefix) / AZ::IO::Path(serviceType);
        settingsRegistry->Get(output, setRegPath.String());
        return output;
    }
} // namespace ROS2SimulationInterfaces::RegistryUtilities
