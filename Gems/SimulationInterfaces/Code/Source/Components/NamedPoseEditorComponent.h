/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <SimulationInterfaces/NamedPose.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    class NamedPoseEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , public LmbrCentral::TagComponentNotificationsBus::Handler
        , public AZ::TransformNotificationBus::Handler
        , public AZ::EntityBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(NamedPoseEditorComponent, NamedPoseEditorComponentTypeId, AzToolsFramework::Components::EditorComponentBase);
        NamedPoseEditorComponent() = default;
        ~NamedPoseEditorComponent() override = default;

        // EditorComponentBase interface overrides ...
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    private:
        // methods to update named pose configuration (name, pose and added tags. Native O3DE component and systems are used to define this
        // data and it needs to be updated in case of change)

        //  LmbrCentral::TagComponentNotificationsBus overrides
        void OnTagAdded(const LmbrCentral::Tag&) override;
        void OnTagRemoved(const LmbrCentral::Tag&) override;
        // AZ::TransformNotificationBus overrides
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;
        // AZ::EntityBus overrides
        void OnEntityNameChanged(const AZStd::string& name) override;
        void UpdateConfiguration();

        NamedPose m_config;
    };
} // namespace SimulationInterfaces
