/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Source/Debug/MachineLearningDebugModule.h>
#include <Source/Debug/MachineLearningDebugSystemComponent.h>

namespace MachineLearning
{
    MachineLearningDebugModule::MachineLearningDebugModule()
        : AZ::Module()
    {
        m_descriptors.insert(m_descriptors.end(), {
            MachineLearningDebugSystemComponent::CreateDescriptor()
        });
    }

    AZ::ComponentTypeList MachineLearningDebugModule::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList
        {
            azrtti_typeid<MachineLearningDebugSystemComponent>()
        };
    }
}

AZ_DECLARE_MODULE_CLASS(Gem_MachineLearning_Debug, MachineLearning::MachineLearningDebugModule);
