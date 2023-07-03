/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Communication/PublisherConfiguration.h>

namespace ROS2
{
    //! Editor Component responsible for a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    class JointsManipulationEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        JointsManipulationEditorComponent();
        ~JointsManipulationEditorComponent() = default;
        AZ_COMPONENT(JointsManipulationEditorComponent, "{BF2F77FD-92FB-4730-92C7-DDEE54F508BF}", AZ::Component);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        PublisherConfiguration m_jointStatePublisherConfiguration;
        AZStd::unordered_map<AZStd::string, float> m_initialPositions;
    };
} // namespace ROS2
