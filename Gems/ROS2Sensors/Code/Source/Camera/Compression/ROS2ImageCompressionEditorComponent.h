/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include "../Compression/ImageCompressionConfiguration.h"

#include <AzCore/Component/Component.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Editor counterpart of ROS2ImageCompressionComponent.
    //! Carries the configuration in the Editor and builds the game component, which is what opens the
    //! ROS 2 publisher. Nothing is published while a level is only being edited.
    class ROS2ImageCompressionEditorComponent : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(
            ROS2ImageCompressionEditorComponent,
            ROS2Sensors::ROS2ImageCompressionEditorComponentTypeId,
            AzToolsFramework::Components::EditorComponentBase);
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        ROS2ImageCompressionEditorComponent() = default;
        explicit ROS2ImageCompressionEditorComponent(const ImageCompressionConfiguration& configuration);
        ~ROS2ImageCompressionEditorComponent() override = default;

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        ImageCompressionConfiguration m_configuration;
    };
} // namespace ROS2Sensors
