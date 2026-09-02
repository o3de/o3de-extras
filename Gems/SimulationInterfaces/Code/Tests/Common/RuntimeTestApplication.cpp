/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RuntimeTestApplication.h"

namespace UnitTest
{
    RuntimeTestApplication::RuntimeTestApplication(AZStd::string applicationName)
        : RuntimeTestApplication(AZStd::move(applicationName), 0, nullptr)
    {
    }

    RuntimeTestApplication::RuntimeTestApplication(AZStd::string applicationName, int argc, char** argv)
        : AzFramework::Application(&argc, &argv)
        , m_applicationName(AZStd::move(applicationName))
    {
    }

    void RuntimeTestApplication::SetSettingsRegistrySpecializations(AZ::SettingsRegistryInterface::Specializations& specializations)
    {
        Application::SetSettingsRegistrySpecializations(specializations);
        specializations.Append("test");
        specializations.Append(m_applicationName);
    }
} // namespace UnitTest
