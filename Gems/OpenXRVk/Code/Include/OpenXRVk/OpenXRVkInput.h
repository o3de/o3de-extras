/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRInput.h>
#include <OpenXRVk/InputDeviceXRController.h>
#include <OpenXRVk/OpenXRVkSpace.h>
#include <OpenXRVk_Platform.h>
#include <Atom/RPI.Public/XR/XRRenderingInterface.h>

namespace OpenXRVk
{
    // Class that will help manage XrActionSet/XrAction
    class Input final
        : public XR::Input
    {
    public:
        AZ_CLASS_ALLOCATOR(Input, AZ::SystemAllocator);
        AZ_RTTI(Input, "{97ADD1FE-27DF-4F36-9F61-683F881F9477}", XR::Input);

        static XR::Ptr<Input> Create();

        //! Sync all the actions and update controller
        //! as well as various tracked space poses
        void PollActions();

        //! Initialize various actions/actions sets and add support for Oculus touch bindings
        AZ::RHI::ResultCode InitInternal() override;

        //! Create controller action spaces
        AZ::RHI::ResultCode InitializeActionSpace(XrSession xrSession);

        //! Attach action sets
        AZ::RHI::ResultCode InitializeActionSets(XrSession xrSession) const;

        //! Update Controller space information
        void LocateControllerSpace(XrTime predictedDisplayTime, XrSpace baseSpace, AZ::u32 handIndex);

        //! Update information for a specific tracked space type (i.e visualizedSpaceType)
        void LocateVisualizedSpace(XrTime predictedDisplayTime, XrSpace space, XrSpace baseSpace, OpenXRVk::SpaceType visualizedSpaceType);

        //! Return Pose data for a controller attached to a hand index
        AZ::RHI::ResultCode GetControllerPose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const;

        //! Return scale for a controller attached to a hand index
        float GetControllerScale(AZ::u32 handIndex) const;

        //! Return Pose data for a tracked space type (i.e visualizedSpaceType)
        AZ::RHI::ResultCode GetVisualizedSpacePose(OpenXRVk::SpaceType visualizedSpaceType, AZ::RPI::PoseData& outPoseData) const;

        //! Get the Squeeze action
        XrAction GetSqueezeAction(AZ::u32 handIndex) const;

        //! Get the Pose action
        XrAction GetPoseAction(AZ::u32 handIndex) const;

        //! Get the Vibration action
        XrAction GetVibrationAction() const;

        //! Get the Quit action
        XrAction GetQuitAction() const;

        //! Get any button state
        bool GetButtonState(const AzFramework::InputChannelId& channelId) const;

        //! Get the X button state
        bool GetXButtonState() const;

        //! Get the Y button state
        bool GetYButtonState() const;

        //! Get the A button state
        bool GetAButtonState() const;

        //! Get the B button state
        bool GetBButtonState() const;

        //! Get the joystick state for x-axis
        float GetXJoyStickState(AZ::u32 handIndex) const;

        //! Get the joystick state for y-axis
        float GetYJoyStickState(AZ::u32 handIndex) const;

        //! Get the Squeeze action
        float GetSqueezeState(AZ::u32 handIndex) const;

        //! Get the Squeeze action
        float GetTriggerState(AZ::u32 handIndex) const;

    private:
        //! Creates an XrAction
        void CreateAction(XrAction& action, XrActionType actionType,
                          const char* actionName, const char* localizedActionName,
                          uint32_t countSubactionPathCount, const XrPath* subActionPaths) const;


        void CreateActionSet(const XrInstance& xrInstance);
        void CreateAllActions(const XrInstance& xrInstance);
        XrAction GetAction(const AzFramework::InputChannelId& channelId) const;

        //! Destroy native objects
        void ShutdownInternal() override;

        XrActionSet m_actionSet{ XR_NULL_HANDLE };

        XrAction m_hapticAction{};
        AZStd::vector<XrActionSuggestedBinding> m_xrActionPaths{};
        AZStd::unordered_map<AzFramework::InputChannelId, AZStd::size_t> m_xrActionIndices{};

        AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> m_handSubactionPath{};
        AZStd::array<XrSpace, AZ::RPI::XRMaxNumControllers> m_handSpace{};
        AZStd::array<float, AZ::RPI::XRMaxNumControllers> m_handScale{ { 1.0f, 1.0f } };
        AZStd::array<XrBool32, AZ::RPI::XRMaxNumControllers> m_handActive{};

        AZStd::array<XrSpaceLocation, AZ::RPI::XRMaxNumControllers> m_handSpaceLocation{};
        AZStd::array<XrSpaceLocation, SpaceType::Count> m_xrVisualizedSpaceLocations{};

        AzFramework::InputDeviceXRController m_xrController{};
        AzFramework::InputDeviceXRController::Implementation* m_xrControllerImpl{};
        bool m_wasQuitPressedLastSync{ false };
    };
}
