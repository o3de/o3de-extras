/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ControllersEditorSystemComponent.h"
#include <ROS2Controllers/ROS2ControllersTypeIds.h>
#include <ROS2ControllersModuleInterface.h>

#include <Manipulation/JointsManipulationEditorComponent.h>
#include <Manipulation/JointsPositionsEditorComponent.h>

namespace ROS2Controllers
{
    class ROS2ControllersEditorModule : public ROS2ControllersModuleInterface
    {
    public:
        AZ_RTTI(ROS2ControllersEditorModule, ROS2ControllersEditorModuleTypeId, ROS2ControllersModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2ControllersEditorModule, AZ::SystemAllocator);

        ROS2ControllersEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    ROS2ControllersEditorSystemComponent::CreateDescriptor(),
                    JointsManipulationEditorComponent::CreateDescriptor(),
                    JointsPositionsEditorComponent::CreateDescriptor(),
                });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2ControllersEditorSystemComponent>(),
            };
        }
    };
} // namespace ROS2Controllers

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME, _Editor), ROS2Controllers::ROS2ControllersEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2Controllers_Editor, ROS2Controllers::ROS2ControllersEditorModule)
#endif
