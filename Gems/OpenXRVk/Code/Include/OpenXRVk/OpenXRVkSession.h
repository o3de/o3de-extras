/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <OpenXRVk_Platform.h>
#include <OpenXRVk/OpenXRVkSpace.h>
#include <XR/XRSession.h>

namespace OpenXRVk
{
    // Class that will help manage XrSession
    class Session final
        : public XR::Session
    {
    public:
        AZ_CLASS_ALLOCATOR(Session, AZ::SystemAllocator, 0);
        AZ_RTTI(Session, "{6C899F0C-9A3D-4D79-8E4F-92AFB67E5EB1}", XR::Session);

        static XR::Ptr<Session> Create();

        //! Print out all the supported Reference space
        void LogReferenceSpaces();

        //! Process session state when it is updated
        void HandleSessionStateChangedEvent(const XrEventDataSessionStateChanged& stateChangedEvent);

        //! Try and poll the next event 
        const XrEventDataBaseHeader* TryReadNextEvent();

        //! Return the native session
        XrSession GetXrSession() const;

        //! Return the Xrspace related to the SpaceType enum
        XrSpace GetXrSpace(SpaceType spaceType) const;

        //////////////////////////////////////////////////////////////////////////
        // XR::Session overrides
        AZ::RHI::ResultCode InitInternal(AZ::RHI::XRSessionDescriptor* descriptor) override;
        bool IsSessionRunning() const override;
        bool IsSessionFocused() const override;
        bool IsRestartRequested() const override;
        bool IsExitRenderLoopRequested() const override;
        void PollEvents() override;
        void LocateControllerSpace(AZ::u32 handIndex) override;
        AZ::RHI::ResultCode GetControllerPose(const AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const override;
        AZ::RHI::ResultCode GetControllerStagePose(const AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const override;
        AZ::RHI::ResultCode GetViewFrontPose(AZ::RPI::PoseData& outPoseData) const override;
        AZ::RHI::ResultCode GetViewLocalPose(AZ::RPI::PoseData& outPoseData) const override;
        float GetControllerScale(AZ::u32 handIndex) const override;
        float GetXButtonState() const override;
        float GetYButtonState() const override;
        float GetAButtonState() const override;
        float GetBButtonState() const override;
        float GetXJoyStickState(const AZ::u32 handIndex) const override;
        float GetYJoyStickState(const AZ::u32 handIndex) const override;
        float GetSqueezeState(const AZ::u32 handIndex) const override;
        float GetTriggerState(const AZ::u32 handIndex) const override;
        //////////////////////////////////////////////////////////////////////////

    private:

        void ShutdownInternal() override;
        void LogActionSourceName(XrAction action, const AZStd::string_view actionName) const;
        
        XrSession m_session = XR_NULL_HANDLE;
        XrSessionState m_sessionState = XR_SESSION_STATE_UNKNOWN;
        XrEventDataBuffer m_eventDataBuffer;
        XrInstance m_xrInstance = XR_NULL_HANDLE;
        XrGraphicsBindingVulkan2KHR m_graphicsBinding{ XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR };

        bool m_sessionRunning = false;
        bool m_exitRenderLoop = false;
        bool m_requestRestart = false;        
    };
}
