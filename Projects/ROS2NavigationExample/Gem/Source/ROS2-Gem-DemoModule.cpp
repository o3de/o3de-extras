/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "ROS2-Gem-DemoSystemComponent.h"

namespace ROS2_Gem_Demo
{
    class ROS2_Gem_DemoModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(ROS2_Gem_DemoModule, "{0ea31247-8ee5-491d-93cb-296a6c91c995}", AZ::Module);
        AZ_CLASS_ALLOCATOR(ROS2_Gem_DemoModule, AZ::SystemAllocator, 0);

        ROS2_Gem_DemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                ROS2_Gem_DemoSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2_Gem_DemoSystemComponent>(),
            };
        }
    };
}// namespace ROS2_Gem_Demo

AZ_DECLARE_MODULE_CLASS(Gem_ROS2_Gem_Demo, ROS2_Gem_Demo::ROS2_Gem_DemoModule)
