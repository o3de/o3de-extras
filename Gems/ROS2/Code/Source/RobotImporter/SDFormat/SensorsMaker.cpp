/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SensorsMaker.h"
#include "SupportedTags.h"

#include <AzCore/Component/Component.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include <sdf/Camera.hh>
#include <sdf/Imu.hh>

namespace ROS2::SDFormat
{
    void SensorsMaker::AddSensor(const AZ::EntityId entityId, const sdf::Sensor* sensor)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);
        sdf::ElementPtr element = nullptr;

        switch (sensor->Type())
        {
        case sdf::SensorType::CAMERA:
        case sdf::SensorType::DEPTH_CAMERA:
        case sdf::SensorType::RGBD_CAMERA:
            {
                auto* cameraSensor = sensor->CameraSensor();
                AZ_Assert(cameraSensor, "sensor is not sdf::SensorType::CAMERA");
                element = cameraSensor->Element();

                CameraSensorConfiguration cameraConfiguration;
                SensorConfiguration sensorConfiguration;

                sensorConfiguration.m_frequency = sensor->UpdateRate();
                cameraConfiguration.m_depthCamera = cameraSensor->HasDepthCamera();
                cameraConfiguration.m_colorCamera = (sensor->Type() != sdf::SensorType::DEPTH_CAMERA) ? true : false;
                cameraConfiguration.m_width = cameraSensor->ImageWidth();
                cameraConfiguration.m_height = cameraSensor->ImageHeight();
                cameraConfiguration.m_verticalFieldOfViewDeg =
                    cameraSensor->HorizontalFov().Degree() * (cameraConfiguration.m_height / cameraConfiguration.m_width);

                // TODO: add distortion parameters, far/near clip, ROS2 topic, format, and more
                // TODO: read required services instead of hardcoding ones (ROS2FrameComponent)

                auto* frameComponent = entity->CreateComponent<ROS2FrameComponent>();
                AZ_Assert(frameComponent, "failed to create ROS2FrameComponent");
                auto* cameraComponent = entity->CreateComponent<ROS2CameraSensorEditorComponent>(sensorConfiguration, cameraConfiguration);
                AZ_Assert(cameraComponent, "failed to create ROS2CameraSensorEditorComponent");
            }
            break;
        default:
            AZ_Warning("AddSensor", false, "Unsupported sensor type, %d", sensor->Type());
            break;
        }

        const auto& supportedOptions = SupportedTags::GetSupportedTags(sensor->Type());
        const auto& unsupportedOptions = Utils::SDFormat::GetUnsupportedOptions(element, supportedOptions);
        if (!unsupportedOptions.empty())
        {
            m_log.append("Unsupported tags in SDFormat model found:\n");
            for (const auto& opt : unsupportedOptions)
            {
                m_log.append("\t");
                m_log.append(opt);
                m_log.append("\n");
            }
        }
    }

    void SensorsMaker::AddSensors(const AZ::EntityId entityId, const sdf::Link* link)
    {
        for (size_t si = 0; si < link->SensorCount(); ++si)
        {
            const auto* sensor = link->SensorByIndex(si);
            AddSensor(entityId, sensor);
        }
    }

    void SensorsMaker::AddSensors(const AZ::EntityId entityId, const sdf::Joint* joint)
    {
        for (size_t si = 0; si < joint->SensorCount(); ++si)
        {
            const auto* sensor = joint->SensorByIndex(si);
            AddSensor(entityId, sensor);
        }
    }

    const AZStd::string& SensorsMaker::GetLog() const
    {
        return m_log;
    }

    void SensorsMaker::ResetLog()
    {
        m_log.clear();
    }
} // namespace ROS2::SDFormat