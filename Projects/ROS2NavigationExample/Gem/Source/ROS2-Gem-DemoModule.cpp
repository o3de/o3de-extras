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

namespace RobotVacuumSample
{
    class RobotVacuumSampleModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(RobotVacuumSampleModule, "{0ea31247-8ee5-491d-93cb-296a6c91c995}", AZ::Module);
        AZ_CLASS_ALLOCATOR(RobotVacuumSampleModule, AZ::SystemAllocator, 0);

        RobotVacuumSampleModule()
            : AZ::Module()
        {
            // Push results of RobotVacuumSampleSystemComponent::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                RobotVacuumSampleSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<RobotVacuumSampleSystemComponent>(),
            };
        }
    };
}// namespace RobotVacuumSample

AZ_DECLARE_MODULE_CLASS(Gem_RobotVacuumSample, RobotVacuumSample::RobotVacuumSampleModule)
