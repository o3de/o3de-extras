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

#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2Sensors/Camera/CameraConfigurationRequestBus.h>
#include <ROS2Sensors/Camera/CameraSensorConfiguration.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! ROS2 Camera Editor sensor component class
    //! Allows turning an entity into a camera sensor in Editor
    //! Component draws camera frustum in the Editor
    class ROS2CameraSensorEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , public CameraConfigurationRequestBus::Handler
        , protected AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        ROS2CameraSensorEditorComponent();
        ROS2CameraSensorEditorComponent(
            const ROS2::SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration);
        ~ROS2CameraSensorEditorComponent() override = default;
        AZ_EDITOR_COMPONENT(ROS2CameraSensorEditorComponent, ROS2CameraSensorEditorComponentTypeId);
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        void Activate() override;
        void Deactivate() override;

        // AzToolsFramework::Components::EditorComponentBase overrides
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // CameraConfigurationRequestBus::Handler overrides ..
        void SetConfiguration(const CameraSensorConfiguration& configuration) override;
        const CameraSensorConfiguration GetConfiguration() override;
        AZ::Matrix3x3 GetCameraMatrix() override;
        float GetVerticalFOV() override;
        void SetVerticalFOV(float value) override;
        int GetWidth() override;
        void SetWidth(int value) override;
        int GetHeight() override;
        void SetHeight(int value) override;
        bool IsColorCamera() override;
        void SetColorCamera(bool value) override;
        bool IsDepthCamera() override;
        void SetDepthCamera(bool value) override;
        float GetNearClipDistance() override;
        void SetNearClipDistance(float value) override;
        float GetFarClipDistance() override;
        void SetFarClipDistance(float value) override;

        // EntityDebugDisplayEventBus::Handler overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        AZStd::pair<AZStd::string, ROS2::TopicConfiguration> MakeTopicConfigurationPair(
            const AZStd::string& topic, const AZStd::string& messageType, const AZStd::string& configName) const;

        ROS2::SensorConfiguration m_sensorConfiguration;
        CameraSensorConfiguration m_cameraConfiguration;
    };
} // namespace ROS2Sensors
