/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <AzCore/Component/Component.h>
#include <AzCore/std/containers/vector.h>

#include "CameraSensor.h"
#include "CameraSensorConfiguration.h"
#include <ROS2/Camera/CameraCalibrationRequestBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>

namespace ROS2
{
    //! ROS2 Camera sensor component class
    //! Allows turning an entity into a camera sensor
    //! Can be parametrized with following values:
    //!   - camera name
    //!   - camera image width and height in pixels
    //!   - camera vertical field of view in degrees
    //! Camera frustum is facing negative Z axis; image plane is parallel to X,Y plane: X - right, Y - up
    class ROS2CameraSensorComponent
        : public ROS2SensorComponentBase<TickBasedSource>
        , public CameraCalibrationRequestBus::Handler
    {
    public:
        ROS2CameraSensorComponent() = default;
        ROS2CameraSensorComponent(const SensorConfiguration& sensorConfiguration, const CameraSensorConfiguration& cameraConfiguration);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        ~ROS2CameraSensorComponent() override = default;

        AZ_COMPONENT(ROS2CameraSensorComponent, "{3C6B8AE6-9721-4639-B8F9-D8D28FD7A071}", SensorBaseType);
        static void Reflect(AZ::ReflectContext* context);

        // AzToolsFramework::Components::EditorComponentBase overrides ..
        void Activate() override;
        void Deactivate() override;

        // CameraCalibrationRequestBus::Handler overrides ...
        AZ::Matrix3x3 GetCameraMatrix() const override;
        int GetWidth() const override;
        int GetHeight() const override;
        float GetVerticalFOV() const override;

    private:
        //! Helper that adds an image source.
        //! @tparam CameraType type of camera sensor (eg 'CameraColorSensor')
        template<typename CameraType>
        void SetImageSource()
        {
            const auto cameraName = GetCameraNameFromFrame(GetEntity());
            const CameraSensorDescription description{ cameraName, GetNamespace(), m_cameraConfiguration, m_sensorConfiguration };
            m_cameraSensor = AZStd::make_shared<CameraType>(description, GetEntityId());
        }
        //! Retrieve camera name from ROS2FrameComponent's FrameID.
        //! @param entity pointer entity that has ROS2FrameComponent.
        [[nodiscard]] AZStd::string GetCameraNameFromFrame(const AZ::Entity* entity) const;

        ///! Requests message publication from camera sensor.
        void FrequencyTick();

        CameraSensorConfiguration m_cameraConfiguration;
        AZStd::string m_frameName;
        AZStd::shared_ptr<CameraSensor> m_cameraSensor;
    };
} // namespace ROS2
