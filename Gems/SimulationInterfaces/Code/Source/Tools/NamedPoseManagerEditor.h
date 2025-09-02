/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/NamedPosesManager.h>

namespace SimulationInterfaces
{
    /// System component for SimulationInterfaces editor
    class NamedPoseManagerEditor
        : public NamedPoseManager
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = NamedPoseManager;

    public:
        AZ_COMPONENT_DECL(NamedPoseManagerEditor);

        static void Reflect(AZ::ReflectContext* context);

        NamedPoseManagerEditor();
        ~NamedPoseManagerEditor();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace SimulationInterfaces
