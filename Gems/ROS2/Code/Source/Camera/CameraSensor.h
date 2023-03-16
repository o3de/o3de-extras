/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <AzCore/std/containers/span.h>
#include <ROS2/ROS2GemUtilities.h>
#include <chrono>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace ROS2
{
    struct CameraConfiguration
    {
        float m_verticalFieldOfViewDeg = 90.0f; //!< Vertical field of view of camera sensor.
        int m_width = 640; //!< Camera image width in pixels.
        int m_height = 480; //!< Camera image height in pixels.
    };

    //! Structure containing all information required to create the camera sensor.
    struct CameraSensorDescription
    {
        //! Constructor to create the description
<<<<<<< HEAD
        //! @param cameraName - name of the camera; used to differentiate cameras in a multi-camera setup.
        //! @param configuration - configuration structure for the camera, defining its characteristics.
        CameraSensorDescription(const AZStd::string& cameraName, const CameraConfiguration& configuration);
=======
        //! @param cameraName - name of the camera; used to differentiate cameras in a multi-camera setup
        //! @param verticalFov - vertical field of view of camera sensor
        //! @param width - camera image width in pixels
        //! @param height - camera image height in pixels
        //! @param entityId - entityId of camera sensor
        CameraSensorDescription(const AZStd::string& cameraName, float verticalFov, int width, int height, AZ::EntityId entityId);
>>>>>>> 594a255 (Rework RGBD sensor. (#117))

        const CameraConfiguration m_cameraConfiguration; //!< Configuration of the camera.
        const AZStd::string m_cameraName; //!< Camera name to differentiate cameras in a multi-camera setup.

<<<<<<< HEAD
        const float m_aspectRatio; //!< Camera image aspect ratio; equal to (width / height).
        const AZ::Matrix4x4 m_viewToClipMatrix; //!< Camera view to clip space transform matrix; derived from other parameters.
        const AZStd::array<double, 9> m_cameraIntrinsics; //!< Camera intrinsics; derived from other parameters.

=======
        const float m_aspectRatio; //!< camera image aspect ratio; equal to (width / height)
        const AZ::Matrix4x4 m_viewToClipMatrix; //!< camera view to clip space transform matrix; derived from other parameters
        const AZStd::array<double, 9> m_cameraIntrinsics; //!< camera intrinsics; derived from other parameters
        const AZ::EntityId m_entityId; //! Entity Id that is owning this sensor.
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
    private:
        AZ::Matrix4x4 MakeViewToClipMatrix() const;
        AZStd::array<double, 9> MakeCameraIntrinsics() const;
        void ValidateParameters() const;
    };

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
        //! @param publishers - ROS2 publishers to publish image in future : color, depth ...
        //! @param header - header with filled message information (frame, timestamp, seq)
        //! @param cameraPose - current camera pose from which the rendering should take place
        virtual void RequestMessagePublication(
            AZStd::span<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> publishers,
            const AZ::Transform& cameraPose,
            const std_msgs::msg::Header& header);

        //! Get the camera sensor description
        [[nodiscard]] const CameraSensorDescription& GetCameraSensorDescription() const;

    private:
        //! Publish Image Message frame from rendering pipeline
        //! @param publisher - ROS2 publisher to publish image in future
        //! @param header - header with filled message information (frame, timestamp, seq)
        //! @param cameraPose - current camera pose from which the rendering should take place
        void RequestMessagePublication(
            std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher,
            const AZ::Transform& cameraPose,
            const std_msgs::msg::Header& header);

        CameraSensorDescription m_cameraSensorDescription;
        AZStd::vector<AZStd::string> m_passHierarchy;
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::Scene* m_scene = nullptr;
        const AZ::Transform AtomToRos{ AZ::Transform::CreateFromQuaternion(
            AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };
        virtual AZStd::string GetPipelineTemplateName() const = 0; //! Returns name of pass template to use in pipeline
        virtual AZStd::string GetPipelineTypeName() const = 0; //! Type of returned data eg Color, Depth, Optical flow

    protected:
<<<<<<< HEAD
        AZ::EntityId m_entityId;
=======
>>>>>>> 594a255 (Rework RGBD sensor. (#117))
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZStd::string m_pipelineName;

        //! Request a frame from the rendering pipeline
        //! @param cameraPose - current camera pose from which the rendering should take place
        //! @param callback - callback function object that will be called when capture is ready.
        //!                   It's argument is readback structure containing, among other things, a captured image
        void RequestFrame(
            const AZ::Transform& cameraPose, AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);

        //! Request an additional frame from the rendering pipeline
        //! @param callback - callback function object that will be called when capture is ready.
        //!                   It's argument is readback structure containing, among other things, a captured image
        //! @param passName - pass name in pipeline, eg `DepthToLinearDepthPass`
        //! @param slotName - slot name in selected pass, eg `Output`
        void RequestAdditionalFrame(
            AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback,
            const AZStd::string& passName,
            const AZStd::string& slotName);

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
        AZStd::string GetPipelineTypeName() const override;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image
    class CameraColorSensor : public CameraSensor
    {
    public:
        CameraColorSensor(const CameraSensorDescription& cameraSensorDescription, const AZ::EntityId& entityId);

    private:
        AZStd::string GetPipelineTemplateName() const override;
        AZStd::string GetPipelineTypeName() const override;
    };

    //! Implementation of camera sensors that runs pipeline which produces color image and readbacks a depth image from pipeline
    class CameraRGBDSensor : public CameraColorSensor
    {
    public:
        CameraRGBDSensor(const CameraSensorDescription& cameraSensorDescription);

        // CameraSensor overrides
        void RequestMessagePublication(
            AZStd::span<std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>>> publishers,
            const AZ::Transform& cameraPose,
            const std_msgs::msg::Header& header) override;

    private:
        void ReadBackDepth(AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);
    };
} // namespace ROS2
