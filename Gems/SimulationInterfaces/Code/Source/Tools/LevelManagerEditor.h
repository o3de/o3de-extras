/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

#include <Clients/LevelManager.h>

namespace SimulationInterfaces
{
    /// System component for SimulationInterfaces editor
    class LevelManagerEditor
        : public LevelManager
        , protected AzToolsFramework::EditorEvents::Bus::Handler
        , private AzToolsFramework::EditorEntityContextNotificationBus::Handler
    {
        using BaseSystemComponent = LevelManager;

    public:
        AZ_COMPONENT_DECL(LevelManagerEditor);

        static void Reflect(AZ::ReflectContext* context);

        LevelManagerEditor();
        ~LevelManagerEditor();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // EditorEntityContextNotificationBus
        void OnStartPlayInEditorBegin() override;
        void OnStopPlayInEditorBegin()override;
    };
} // namespace SimulationInterfaces
