/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <MachineLearning/MachineLearningTypeIds.h>
#include <MachineLearningModuleInterface.h>
#include "MachineLearningSystemComponent.h"

namespace MachineLearning
{
    class MachineLearningModule
        : public MachineLearningModuleInterface
    {
    public:
        AZ_RTTI(MachineLearningModule, MachineLearningModuleTypeId, MachineLearningModuleInterface);
        AZ_CLASS_ALLOCATOR(MachineLearningModule, AZ::SystemAllocator);
    };
}// namespace MachineLearning

AZ_DECLARE_MODULE_CLASS(Gem_MachineLearning, MachineLearning::MachineLearningModule)
