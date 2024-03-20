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
#include <ROS2/Communication/TopicConfiguration.h>

namespace ROS2
{
    //! Editor Component responsible for a hierarchical system of joints such as robotic arm with Articulations or Hinge Joints.
    class JointsPositionsEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        JointsPositionsEditorComponent();
        ~JointsPositionsEditorComponent() = default;
        AZ_EDITOR_COMPONENT(JointsPositionsEditorComponent, "{6fef3251-f910-4bf7-a4e6-1cf4f73daaf8}");

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        AZ::Crc32 FindAllJoints();

        TopicConfiguration m_topicConfiguration; //!< Configuration of the subscribed topic.
        AZStd::vector<AZStd::string> m_jointNames; //!< Ordered list of joint names that can be modified via subscriber
    };
} // namespace ROS2
