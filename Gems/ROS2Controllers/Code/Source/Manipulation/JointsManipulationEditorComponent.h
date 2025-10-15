/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/utility/pair.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2Controllers/ROS2ControllersTypeIds.h>

namespace ROS2Controllers
{
    //! Editor Component responsible for a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    class JointsManipulationEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        JointsManipulationEditorComponent();
        JointsManipulationEditorComponent(const ROS2::PublisherConfiguration& publisherConfiguration);
        ~JointsManipulationEditorComponent() = default;
        AZ_EDITOR_COMPONENT(JointsManipulationEditorComponent, JointsManipulationEditorComponentTypeId);

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        AZ::Crc32 ReloadJoints();

        ROS2::PublisherConfiguration m_jointStatePublisherConfiguration;
        AZStd::vector<AZStd::pair<AZStd::string, float>> m_initialPositions;
    };
} // namespace ROS2Controllers
