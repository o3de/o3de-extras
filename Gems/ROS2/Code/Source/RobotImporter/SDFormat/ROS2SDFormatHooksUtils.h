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
#include <ROS2/RobotImporter/SDFormatModelPluginImporterHook.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <Source/EditorArticulationLinkComponent.h>
#include <Source/EditorHingeJointComponent.h>

#include <sdf/Model.hh>
#include <sdf/Sensor.hh>

namespace ROS2::SDFormat
{
    namespace HooksUtils
    {
        //! Add a ROS 2 topic configuration to sensor parameters.
        //! @param sensorConfig sensor's configuration which hosts multiple topic configurations
        //! @param topic ROS 2 topic name
        //! @param messageType ROS 2 message type
        //! @param configName name under which topic configuration is stored in sensor's configuration
        void AddTopicConfiguration(
            SensorConfiguration& sensorConfig,
            const AZStd::string& topic,
            const AZStd::string& messageType,
            const AZStd::string& configName);

        //! Find O3DE entity id of the SDFormat joint based on its name and a map of all created entities.
        //! @param jointName name of the joint in query
        //! @param sdfModel reference to SDFormat model
        //! @param createdEntities list of all created entities passed as entity creation results
        //! @return entity id (invalid id if not found)
        AZ::EntityId GetJointEntityId(const std::string& jointName, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities);

        //! Enable motor in EditorHingeJointComponent if possible
        //! @param entityId entity id of the modified entity
        void EnableMotor(const AZ::EntityId& entityId);

        //! Set transform to entity containing sensor based on <pose> tag in sdformat data (if available).
        //! @param entity entity to which the transform is set
        //! @param sdfSensor reference to SDFormat sensor possibly containing <pose> tag
        void SetSensorEntityTransform(AZ::Entity& entity, const sdf::Sensor& sdfSensor);

        //! Create a component and attach the component to the entity.
        //! This method ensures that game components are wrapped into GenericComponentWrapper.
        //! @param entity entity to which the new component is added
        //! @param args constructor arguments used to create the new component
        //! @return A pointer to the component. Returns a null pointer if the component could not be created.
        template<class ComponentType, typename... Args>
        AZ::Component* CreateComponent(AZ::Entity& entity, Args&&... args)
        {
            // Do not create a component if the same type is already added.
            if (entity.FindComponent<ComponentType>())
            {
                return nullptr;
            }

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
                if (!entity.IsComponentReadyToAdd(component) || !entity.AddComponent(component))
                {
                    delete component;
                    component = nullptr;
                }
            }
            return component;
        }

        //! Create a component and attach the component to the entity.
        //! This method ensures that game components are wrapped into GenericComponentWrapper.
        //! @param entityId entity id to which the new component is added
        //! @param args constructor arguments used to create the new component
        //! @return A pointer to the component. Returns a null pointer if the component could not be created.
        template<class ComponentType, typename... Args>
        AZ::Component* CreateComponent(const AZ::EntityId& entityId, Args&&... args)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            if (entity != nullptr)
            {
                return CreateComponent<ComponentType>(*entity, AZStd::forward<Args>(args)...);
            }

            return nullptr;
        }
    } // namespace HooksUtils
} // namespace ROS2::SDFormat
