/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>
#include <AzFramework/Application/Application.h>

namespace UnitTest
{
    class RuntimeTestApplication
        : public AzFramework::Application
    {
    public:
        AZ_CLASS_ALLOCATOR(RuntimeTestApplication, AZ::SystemAllocator)
        explicit RuntimeTestApplication(AZStd::string applicationName);
        RuntimeTestApplication(AZStd::string applicationName, int argc, char** argv);

        void SetSettingsRegistrySpecializations(AZ::SettingsRegistryInterface::Specializations& specializations) override;

    protected:
        AZStd::string m_applicationName;
    };
} // namespace UnitTest
