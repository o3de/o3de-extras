/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Module/Module.h>

namespace MachineLearning
{
    class MachineLearningDebugModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(MachineLearningDebugModule, "{79F87F90-31FC-4A5D-B0B3-9CB8F0CA93E2}", AZ::Module);
        AZ_CLASS_ALLOCATOR(MachineLearningDebugModule, AZ::SystemAllocator);

        MachineLearningDebugModule();
        ~MachineLearningDebugModule() override = default;

        AZ::ComponentTypeList GetRequiredSystemComponents() const override;
    };
}
