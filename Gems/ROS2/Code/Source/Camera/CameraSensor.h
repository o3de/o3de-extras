/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2/ROS2GemUtilities.h"
#include <Atom/Feature/Utils/FrameCaptureBus.h>
#include <chrono>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

class Entity;

namespace ROS2
{

    //! Structure containing all information required to create the camera sensor
    struct CameraSensorDescription
    {
        //! Constructor to create the description
        //! @param cameraName - name of the camera; used to differentiate cameras in a multi-camera setup
        //! @param verticalFov - vertical field of view of camera sensor
        //! @param width - camera image width in pixels
        //! @param height - camera image height in pixels
        CameraSensorDescription(const AZStd::string& cameraName, float verticalFov, int width, int height);

        const float m_verticalFieldOfViewDeg; //!< camera vertical field of view
        double m_verticalFieldOfViewRad;
        const int m_width; //!< camera image width in pixels
        const int m_height; //!< camera image height in pixels
        const AZStd::string m_cameraName; //!< camera name to differentiate cameras in a multi-camera setup

        const float m_aspectRatio; //!< camera image aspect ratio; equal to (width / height)
        const AZ::Matrix4x4 m_viewToClipMatrix; //!< camera view to clip space transform matrix; derived from other parameters
        const AZStd::array<double, 9> m_cameraIntrinsics; //!< camera intrinsics; derived from other parameters

    private:
        AZ::Matrix4x4 MakeViewToClipMatrix() const;

        AZStd::array<double, 9> MakeCameraIntrinsics() const;

        void validateParameters() const;
    };

    //! Class to create camera sensor using Atom renderer
    //! It creates dedicated rendering pipeline for each camera
    class CameraSensor
    {
    public:
        //! Initializes rendering pipeline for the camera sensor
        //! @param cameraSensorDescription - camera sensor description used to create camera pipeline
        CameraSensor(const CameraSensorDescription& cameraSensorDescription);

        //! Deinitializes rendering pipeline for the camera sensor
        virtual ~CameraSensor();

        //! Function publish Image Message frame from rendering pipeline
        //! @param publisher - ROS2 publisher to publish image in future
        //! @param header - header with filled message information (frame, timestamp, seq)
        //! @param cameraPose - current camera pose from which the rendering should take place
        void publishMassage(
            std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> publisher,
            const AZ::Transform& cameraPose,
            const std_msgs::msg::Header& header);

        //! Function to get camera sensor description
        [[nodiscard]] const CameraSensorDescription& GetCameraSensorDescription() const;

    private:
        //! Function requesting frame from rendering pipeline
        //! @param cameraPose - current camera pose from which the rendering should take place
        //! @param callback - callback function object that will be called when capture is ready
        //!                   it's argument is readback structure containing, among other thins, captured image
        void RequestFrame(
            const AZ::Transform& cameraPose, AZStd::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult& result)> callback);

        CameraSensorDescription m_cameraSensorDescription;
        AZStd::vector<AZStd::string> m_passHierarchy;
        AZ::RPI::RenderPipelinePtr m_pipeline;
        AZ::RPI::ViewPtr m_view;
        AZ::RPI::Scene* m_scene = nullptr;
        const AZ::Transform kAtomToRos{ AZ::Transform::CreateFromQuaternion(
            AZ::Quaternion::CreateFromMatrix3x3(AZ::Matrix3x3::CreateFromRows({ 1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }))) };
        virtual AZStd::string getPipelineTemplateName() = 0;
        virtual AZStd::string getPipelineTypeName() = 0;

    protected:
        //! Read and setup Atom Passes
        void setupPasses();
    };

    class CameraDepthSensor : public CameraSensor
    {
    public:
        CameraDepthSensor(const CameraSensorDescription& cameraSensorDescription);

    private:
        virtual AZStd::string getPipelineTemplateName() override;
        virtual AZStd::string getPipelineTypeName() override;
    };

    class CameraColorSensor : public CameraSensor
    {
    public:
        CameraColorSensor(const CameraSensorDescription& cameraSensorDescription);

    private:
        virtual AZStd::string getPipelineTemplateName() override;
        virtual AZStd::string getPipelineTypeName() override;
    };

} // namespace ROS2
