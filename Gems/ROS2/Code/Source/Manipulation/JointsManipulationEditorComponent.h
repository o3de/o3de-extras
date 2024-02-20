/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "JointNamePositionPair.h"
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/Communication/SubscriberConfiguration.h>
#include <AzCore/Serialization/Json/BaseJsonSerializer.h>

namespace ROS2
{
    // Custom JSON serializer for JointsManipulationEditorComponent configuration to handle version conversion
    class JointsManipulationEditorComponentConfigSerializer : public AZ::BaseJsonSerializer
    {
    public:
        AZ_RTTI(ROS2::JointsManipulationEditorComponentConfigSerializer, "{8b884dae-82b7-4514-a2b4-fd2e7fc4a4fb}", AZ::BaseJsonSerializer);
        AZ_CLASS_ALLOCATOR_DECL;

        AZ::JsonSerializationResult::Result Load(
            void* outputValue,
            const AZ::Uuid& outputValueTypeId,
            const rapidjson::Value& inputValue,
            AZ::JsonDeserializerContext& context) override;
    };

    //! Editor Component responsible for a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    class JointsManipulationEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
        friend class JointsManipulationEditorComponentConfigSerializer;

    public:
        JointsManipulationEditorComponent();
        ~JointsManipulationEditorComponent() = default;
        AZ_EDITOR_COMPONENT(JointsManipulationEditorComponent, "{BF2F77FD-92FB-4730-92C7-DDEE54F508BF}");

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void Reflect(AZ::ReflectContext* context);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        PublisherConfiguration m_jointStatePublisherConfiguration;
        SubscriberConfiguration m_jointPositionsSubscriberConfiguration;
        AZStd::vector<JointNamePositionPair> m_initialPositions;
    };
} // namespace ROS2
