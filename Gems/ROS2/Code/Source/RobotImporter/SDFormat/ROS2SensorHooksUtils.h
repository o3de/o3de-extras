/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2::SDFormat
{
    namespace ROS2SensorHooks
    {
        namespace Utils
        {
            //! Add a ROS2 topic configuration to sensor parameters.
            //! @param sensorConfig sensor's configuration which hosts multiple topic configurations
            //! @param topic ROS2 topic name
            //! @param messageType ROS2 message type
            //! @param configName name under which topic configuration is stored in sensor's configuration
            void AddTopicConfiguration(
                SensorConfiguration& sensorConfig,
                const AZStd::string& topic,
                const AZStd::string& messageType,
                const AZStd::string& configName);

            //! Create a component and attach the component to the entity.
            //! This method ensures that game components are wrapped into GenericComponentWrapper.
            //! @param entity entity to which the new component is added
            //! @param args constructor arguments used to create the new component
            //! @return A pointer to the component. Returns a null pointer if the component could not be created.
            template<class ComponentType, typename... Args>
            AZ::Component* CreateComponent(AZ::Entity& entity, Args&&... args)
            {
                // Create component.
                // If it's not an "editor component" then wrap it in a GenericComponentWrapper.
                AZ::Component* component = nullptr;
                if (AZ::GetRttiHelper<ComponentType>() &&
                    AZ::GetRttiHelper<ComponentType>()->IsTypeOf(AzToolsFramework::Components::EditorComponentBase::RTTI_Type()))
                {
                    component = aznew ComponentType(AZStd::forward<Args>(args)...);
                }
                else
                {
                    AZ::Component* gameComponent = aznew ComponentType(AZStd::forward<Args>(args)...);
                    component = aznew AzToolsFramework::Components::GenericComponentWrapper(gameComponent);
                }
                AZ_Assert(component, "Failed to create component: %s", AZ::AzTypeInfo<ComponentType>::Name());

                if (component)
                {
                    if (!entity.AddComponent(component))
                    {
                        delete component;
                        component = nullptr;
                    }
                }
                return component;
            }
        } // namespace Utils
    } // namespace ROS2SensorHooks
} // namespace ROS2::SDFormat
