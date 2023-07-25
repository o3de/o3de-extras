/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SensorsMaker.h"

#include <AzCore/Component/Component.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>

#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>

#include <sdf/Camera.hh>
#include <sdf/Imu.hh>
#include <sdf/Sensor.hh>

namespace ROS2::SDFormat
{
    void AddSensor(AZ::EntityId entityId, const sdf::Sensor* sensor)
    {
        AZ::Entity* entity = AzToolsFramework::GetEntityById(entityId);

        switch (sensor->Type())
        {
        case sdf::SensorType::CAMERA:
        case sdf::SensorType::DEPTH_CAMERA:
        case sdf::SensorType::RGBD_CAMERA:
            {
                auto* cameraSensor = sensor->CameraSensor();
                AZ_Assert(cameraSensor, "sensor is not sdf::SensorType::CAMERA");

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
                // TODO: read required services instead of hardcoding ones

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
    }

    void SensorsMaker::AddSensors(AZ::EntityId entityId, const sdf::Link* link) const
    {
        for (size_t si = 0; si < link->SensorCount(); ++si)
        {
            const auto* sensor = link->SensorByIndex(si);
            AddSensor(entityId, sensor);
        }
    }

    void SensorsMaker::AddSensors(AZ::EntityId entityId, const sdf::Joint* joint) const
    {
        for (size_t si = 0; si < joint->SensorCount(); ++si)
        {
            const auto* sensor = joint->SensorByIndex(si);
            AddSensor(entityId, sensor);
        }
    }
} // namespace ROS2::SDFormat