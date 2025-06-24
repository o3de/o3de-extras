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
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <AzToolsFramework/ToolsComponents/GenericComponentWrapper.h>
#include <Clients/ROS2SensorsSystemComponent.h>
#include <ROS2Sensors/ROS2SensorsEditorBus.h>

namespace ROS2Sensors
{
    /// System component for ROS2Sensors editor
    class ROS2SensorsEditorSystemComponent
        : public ROS2SensorsSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
        , protected ROS2SensorsEditorRequestBus::Handler
    {
        using BaseSystemComponent = ROS2SensorsSystemComponent;

    public:
        AZ_COMPONENT_DECL(ROS2SensorsEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2SensorsEditorSystemComponent();
        ~ROS2SensorsEditorSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ROS2SensorsEditorRequestBus interface implementation
        AZ::Component* CreateROS2GnssSensorComponent(AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration) override;
        AZ::Component* CreateROS2CameraSensorComponent(
            AZ::Entity& entity,
            const ROS2::SensorConfiguration& sensorConfiguration,
            const CameraSensorConfiguration& cameraConfiguration) override;
        AZ::Component* CreateROS2ImuSensorComponent(
            AZ::Entity& entity,
            const ROS2::SensorConfiguration& sensorConfiguration,
            const ImuSensorConfiguration& imuConfiguration) override;
        AZ::Component* CreateROS2LidarSensorComponent(
            AZ::Entity& entity,
            const ROS2::SensorConfiguration& sensorConfiguration,
            const LidarSensorConfiguration& lidarConfiguration) override;
        AZ::Component* CreateROS2Lidar2DSensorComponent(
            AZ::Entity& entity,
            const ROS2::SensorConfiguration& sensorConfiguration,
            const LidarSensorConfiguration& lidar2DConfiguration) override;
        AZ::Component* CreateROS2OdometrySensorComponent(AZ::Entity& entity, const ROS2::SensorConfiguration& sensorConfiguration) override;
        ////////////////////////////////////////////////////////////////////////

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

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
    };
} // namespace ROS2Sensors
