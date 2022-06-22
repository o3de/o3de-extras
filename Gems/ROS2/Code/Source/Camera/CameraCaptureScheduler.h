/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>
#include <Atom/Feature/Utils/FrameCaptureBus.h>

#include <Clock/SimulationClock.h>

#include <Atom/RPI.Public/RenderPipeline.h>

#include <list>

namespace ROS2 {

//! Singleton class to initialize and synchronize frame captures for multiple cameras
//! It supports the following features:
//! - only one frame is rendered and captured at a time
//! - scheduler waits for a notification that capture is finished
//! - queue with camera captures collects only one capture request per camera
//! @note This is a temporary solution until the multi camera continuous capture is implemented on Atom side
//! @note Components are synchronous by default, so no multithreading guards were placed
class CameraCaptureScheduler : public AZ::Render::FrameCaptureNotificationBus::Handler {
public:
    //! Get underlying singleton
    static CameraCaptureScheduler& Get();

    //! Requests frame rendering if applicable, sets callback and adds to rendering queue is necessary
    //! Calling this function does not guarantee start of the rendering.
    //!
    //! @note Pass hierarchy is expected to have two elements, where first is the unique pipeline name
    bool TryRequestFrame(const AZStd::vector <AZStd::string>& passHierarchy,
                         std::function<void(
                              const AZ::RPI::AttachmentReadback::ReadbackResult &result)> frameRenderedCallback,
                         AZ::RPI::RenderPipelinePtr pipeline);

private:
    void OnCaptureFinished(AZ::Render::FrameCaptureResult result, const AZStd::string &info) override;

    CameraCaptureScheduler();

    ~CameraCaptureScheduler() override;

    struct CameraCaptureRequest {
        AZStd::vector<AZStd::string> pipelineHierarchy;
        std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult &result)> frameRenderedCallback;
        AZ::RPI::RenderPipelinePtr pipeline;

        //! Compares only the pipeline name
        bool operator==(const CameraCaptureRequest &rhs) const;
        CameraCaptureRequest(
                AZStd::vector<AZStd::string> pipelineHierarchy,
                std::function<void(
                        const AZ::RPI::AttachmentReadback::ReadbackResult &result)> frameRenderedCallback,
                AZ::RPI::RenderPipelinePtr pipeline);
        CameraCaptureRequest(CameraCaptureRequest &&other) = default;
        CameraCaptureRequest &operator=(CameraCaptureRequest&& other) = default;
    };

    bool captureInProgres = false;
    std::list<CameraCaptureRequest> captureRequestQueue;
};
}
