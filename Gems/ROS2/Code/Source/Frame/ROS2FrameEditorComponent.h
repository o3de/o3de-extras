/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Frame/ROS2FrameSystemBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    //! Editor component for ROS2 frame management in the O3DE Editor.
    //!
    //! This component provides the editor-specific functionality for ROS2 frames,
    //! including real-time namespace updates
    //! and configuration management.
    //!
    //! @note An entity can only have a single ROS2FrameEditorComponent.
    class ROS2FrameEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , public ROS2FrameComponentBus::Handler
        , public AZ::EntityBus::Handler
        , public ROS2FrameInternalComponentBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FrameEditorComponent, ROS2FrameEditorComponentTypeId, AzToolsFramework::Components::EditorComponentBase);

        ROS2FrameEditorComponent() = default;
        ~ROS2FrameEditorComponent() = default;
        ROS2FrameEditorComponent(const ROS2FrameConfiguration ros2FrameConfiguration);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Get a copy of the current component configuration.
        //! @return Current frame configuration
        ROS2FrameConfiguration GetConfiguration() const;

    private:
        // ROS2FrameEditorComponentBus::Handler overrides
        AZStd::string GetNamespacedFrameID() const override;
        AZ::Name GetNamespacedJointName() const override;
        AZStd::string GetNamespace() const override;
        void UpdateNamespace(const AZStd::string& parentNamespace) override;
        AZStd::string GetGlobalFrameName() const override;
        bool IsTopLevel() const override;
        bool IsDynamic() const override;
        AZ::EntityId GetFrameParent() const override;
        AZStd::set<AZ::EntityId> GetFrameChildren() const override;
        void SetJointName(const AZStd::string& jointName) override;
        void SetFrameID(const AZStd::string& frameId) override;
        void SetConfiguration(const ROS2FrameConfiguration& configuration) override;

        // AZ::EntityBus::Handler override
        void OnEntityNameChanged(const AZStd::string& name) override;

        AZ::Crc32 OnFrameConfigurationChange();

        ROS2FrameConfiguration m_configuration;
    };
} // namespace ROS2
