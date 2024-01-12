/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "CameraPublishers.h"
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <AzCore/std/containers/span.h>
#include <ROS2/ROS2GemUtilities.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace ROS2
{
    //! Class to create camera sensor using Atom renderer
    //! It creates dedicated rendering pipeline for each camera
    class CameraSensor
    {
    public:
        //! Initializes rendering pipeline for the camera sensor.
        //! @param cameraSensorDescription - camera sensor description used to create camera pipeline.
        //! @param entityId - entityId for the owning sensor component.
        CameraSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId);

        //! Deinitializes rendering pipeline for the camera sensor
        virtual ~CameraSensor();

        //! Publish Image Message frame from rendering pipeline
        //! @param cameraPose - current camera pose from which the rendering should take place
        //! @param header - header with filled message information (frame, timestamp, seq)
        virtual void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header);

        //! Get the camera sensor description
        [[nodiscard]] const CameraSensorDescription& GetCameraSensorDescription() const;

    private:
        AZStd::vector<AZStd::string> m_passHierarchy;
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::Scene* m_scene = nullptr;
        const AZ::Transform AtomToRos{ AZ::Transform::CreateFromQuaternion(
            AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };
        virtual AZStd::string GetPipelineTemplateName() const = 0; //! Returns name of pass template to use in pipeline
        virtual CameraSensorDescription::CameraChannelType GetChannelType()
            const = 0; //! Type of returned data eg Color, Depth, Optical flow

    protected:
        CameraSensorDescription m_cameraSensorDescription;
        CameraPublishers m_cameraPublishers;
        AZ::EntityId m_entityId;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZStd::string m_pipelineName;

        //! Request a frame from the rendering pipeline
        //! @param cameraPose - current camera pose from which the rendering should take place
        //! @param callback - callback function object that will be called when capture is ready.
        //!                   It's argument is readback structure containing, among other things, a captured image
        void RequestFrame(
            const AZ::Transform& cameraPose, AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);

        //! Read and setup Atom Passes
        void SetupPasses();
    };

    //! Implementation of camera sensors that runs pipeline which produces depth image
    class CameraDepthSensor : public CameraSensor
    {
    public:
        CameraDepthSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId);

    private:
        AZStd::string GetPipelineTemplateName() const override;
        CameraSensorDescription::CameraChannelType GetChannelType() const override;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image
    class CameraColorSensor : public CameraSensor
    {
    public:
        CameraColorSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId);

    private:
        AZStd::string GetPipelineTemplateName() const override;
        CameraSensorDescription::CameraChannelType GetChannelType() const override;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image and readbacks a depth image from pipeline
    class CameraRGBDSensor : public CameraColorSensor
    {
    public:
        CameraRGBDSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId);

        // CameraSensor overrides
        void RequestMessagePublication(const AZ::Transform& cameraPose, const std_msgs::msg::Header& header) override;

    private:
        void ReadBackDepth(AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);
    };
} // namespace ROS2
