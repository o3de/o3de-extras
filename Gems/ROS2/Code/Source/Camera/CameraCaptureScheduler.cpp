/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "Camera/CameraCaptureScheduler.h"

namespace ROS2 {

CameraCaptureScheduler& CameraCaptureScheduler::Get() {
    static CameraCaptureScheduler i;
    return i;
}

CameraCaptureScheduler::CameraCaptureScheduler() {
    AZ::Render::FrameCaptureNotificationBus::Handler::BusConnect();
}

CameraCaptureScheduler::~CameraCaptureScheduler() {
    AZ::Render::FrameCaptureNotificationBus::Handler::BusDisconnect();
}

bool CameraCaptureScheduler::TryRequestFrame(const AZStd::vector <AZStd::string>& passHierarchy,
                                             std::function<void(const AZ::RPI::AttachmentReadback::ReadbackResult
                                                           &result)> frameRenderedCallback,
                                             AZ::RPI::RenderPipelinePtr pipeline) {
    if (passHierarchy.size() != 2) {
        AZ_TracePrintf("CameraCaptureScheduler", "Pipeline hierarchy must have exactly size of 2. Has %lu\n",
                       passHierarchy.size());
        return false;
    }

    if (!pipeline) {
        AZ_TracePrintf("CameraCaptureScheduler", "Pipeline is nullptr\n");
        return false;
    }

    if (!frameRenderedCallback) {
        AZ_TracePrintf("CameraCaptureScheduler", "Callback is empty\n");
        return false;
    }

    CameraCaptureRequest request(passHierarchy, frameRenderedCallback, pipeline);
    auto found_it = std::find(captureRequestQueue.cbegin(), captureRequestQueue.cend(), request);
    if (found_it != captureRequestQueue.cend()) {
        return false;
    }

    captureRequestQueue.emplace_back(std::move(request));

    if (captureInProgres) {
        return false;
    }

    auto callback = [this](const AZ::RPI::AttachmentReadback::ReadbackResult &result) {
        auto currentRequest = captureRequestQueue.begin();
        currentRequest->frameRenderedCallback(result);
    };

    pipeline->AddToRenderTickOnce();
    AZ::Render::FrameCaptureRequestBus::Broadcast(
            &AZ::Render::FrameCaptureRequestBus::Events::CapturePassAttachmentWithCallback,
            passHierarchy,
            AZStd::string("Output"),
            callback,
            AZ::RPI::PassAttachmentReadbackOption::Output);
    captureInProgres = true;
    return true;
}

void CameraCaptureScheduler::OnCaptureFinished(AZ::Render::FrameCaptureResult result, const AZStd::string &info) {
    captureInProgres = false;
    captureRequestQueue.erase(captureRequestQueue.begin());
}

bool CameraCaptureScheduler::CameraCaptureRequest::operator==(const CameraCaptureScheduler::CameraCaptureRequest &s) const {
    return pipelineHierarchy[0] == s.pipelineHierarchy[0];
}

CameraCaptureScheduler::CameraCaptureRequest::CameraCaptureRequest(
        AZStd::vector<AZStd::string> pipelineHierarchy,
        std::function<void(
                const AZ::RPI::AttachmentReadback::ReadbackResult &result)> frameRenderedCallback,
        AZ::RPI::RenderPipelinePtr pipeline)
    : pipelineHierarchy(std::move(pipelineHierarchy))
    , frameRenderedCallback(std::move(frameRenderedCallback))
    , pipeline(std::move(pipeline)) {}
}
