/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include "CameraSensorConfiguration.h"
#include <ROS2/Camera/CameraCalibrationRequestBus.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    //! ROS2 Camera Editor sensor component class
    //! Allows turning an entity into a camera sensor in Editor
    //! Component draws camera frustum in the Editor
    class ROS2CameraSensorEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , public CameraCalibrationRequestBus::Handler
        , protected AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        ROS2CameraSensorEditorComponent();
        ROS2CameraSensorEditorComponent(
            const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration);
        ~ROS2CameraSensorEditorComponent() override = default;
        AZ_EDITOR_COMPONENT(ROS2CameraSensorEditorComponent, "{3C2A86B2-AD58-4BF1-A5EF-71E0F94A2B42}");
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        void Activate() override;
        void Deactivate() override;

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        // CameraCalibrationRequestBus::Handler overrides
        AZ::Matrix3x3 GetCameraMatrix() const override;
        int GetWidth() const override;
        int GetHeight() const override;
        float GetVerticalFOV() const override;

    private:
        // EntityDebugDisplayEventBus::Handler overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        AZStd::pair<AZStd::string, TopicConfiguration> MakeTopicConfigurationPair(
            const AZStd::string& topic, const AZStd::string& messageType, const AZStd::string& configName) const;

        SensorConfiguration m_sensorConfiguration;
        CameraSensorConfiguration m_cameraSensorConfiguration;
    };
} // namespace ROS2
