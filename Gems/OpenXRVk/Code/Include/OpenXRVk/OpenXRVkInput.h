/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRInput.h>
#include <OpenXRVk/OpenXRVkSpace.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    // Class that will help manage XrActionSet/XrAction
    class Input final
        : public XR::Input
    {
    public:
        AZ_CLASS_ALLOCATOR(Input, AZ::SystemAllocator, 0);
        AZ_RTTI(Input, "{97ADD1FE-27DF-4F36-9F61-683F881F9477}", XR::Input);

        static XR::Ptr<Input> Create();
    
        //! Sync all the actions and update controller
        //! as well as various tracked space poses 
        void PollActions() override;

        //! Initialize various actions/actions sets and add support for Oculus touch bindings
        AZ::RHI::ResultCode InitInternal() override;

        //! Create controller action spaces
        AZ::RHI::ResultCode InitializeActionSpace(XrSession xrSession);

        //! Attach action sets
        AZ::RHI::ResultCode InitializeActionSets(XrSession xrSession);

        //! Update Controller space information
        void LocateControllerSpace(XrTime predictedDisplayTime, XrSpace baseSpace, AZ::u32 handIndex);

        //! Update information for a specific tracked space type (i.e visualizedSpaceType)
        void LocateVisualizedSpace(XrTime predictedDisplayTime, XrSpace space, XrSpace baseSpace, OpenXRVk::SpaceType visualizedSpaceType);

        //! Return Pose data for a controller attached to a view index
        AZ::RHI::ResultCode GetControllerPose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const;

        //! Return scale for a controller attached to a view index
        float GetControllerScale(AZ::u32 viewIndex) const;

        //! Return Pose data for a tracked space type (i.e visualizedSpaceType)
        AZ::RHI::ResultCode GetVisualizedSpacePose(OpenXRVk::SpaceType visualizedSpaceType, AZ::RPI::PoseData& outPoseData) const;

        //! Get the Pose action
        XrAction GetSqueezeAction() const;

        //! Get the Pose action
        XrAction GetPoseAction() const;

        //! Get the Vibration action
        XrAction GetVibrationAction() const;

        //! Get the Quit action
        XrAction GetQuitAction() const;

        //! Get the X button state
        float GetXButtonState() const;

        //! Get the Y button state
        float GetYButtonState() const;

        //! Get the A button state
        float GetAButtonState() const;

        //! Get the B button state
        float GetBButtonState() const;

        //! Get the joystick state for x-axis
        float GetXJoyStickState(AZ::u32 handIndex) const;

        //! Get the joystick state for y-axis
        float GetYJoyStickState(AZ::u32 handIndex) const;

        //! Get the Squeeze action
        float GetSqueezeState(AZ::u32 handIndex) const;

        //! Get the Squeeze action
        float GetTriggerState(AZ::u32 handIndex) const;

    private:

        struct SingleActionData
        {
            XrAction m_actionHandle{ XR_NULL_HANDLE };
            float m_actionState = 0.0f;
        };

        struct DualActionData
        {
            XrAction m_actionHandle{ XR_NULL_HANDLE };
            AZStd::array<float, AZ::RPI::XRMaxNumControllers> m_actionState = { { 0.0f, 0.0f } };
        };

        struct ControllerActionData
        {
            SingleActionData m_actionData;
            uint16_t m_handIndex = 0;
        };

        //! Create a XrAction
        void CreateAction(XrAction& action, XrActionType actionType,
                          const char* actionName, const char* localizedActionName,
                          uint32_t countSubactionPathCount, const XrPath* subActionPaths);

        //! Destroy native objects
        void ShutdownInternal() override;

        bool GetActionState(XrSession xrSession, XrAction xrAction, uint16_t handIndex, float& outputSate);
        bool UpdateActionState(XrSession xrSession, SingleActionData& actionData, uint16_t handIndex);
        bool UpdateActionState(XrSession xrSession, DualActionData& actionData, uint16_t handIndex);

        XrActionSet m_actionSet{ XR_NULL_HANDLE };
        XrAction m_poseAction{ XR_NULL_HANDLE };
        XrAction m_vibrateAction{ XR_NULL_HANDLE };
        XrAction m_quitAction{ XR_NULL_HANDLE };
        DualActionData m_squeezeAction;
        DualActionData m_triggerAction;

        AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> m_handSubactionPath;
        AZStd::array<XrSpace, AZ::RPI::XRMaxNumControllers> m_handSpace;
        AZStd::array<float, AZ::RPI::XRMaxNumControllers> m_handScale = { { 1.0f, 1.0f } };
        AZStd::array<XrBool32, AZ::RPI::XRMaxNumControllers> m_handActive;

        AZStd::array<XrSpaceLocation, AZ::RPI::XRMaxNumControllers> m_handSpaceLocation;
        AZStd::array<XrSpaceLocation, SpaceType::Count> m_xrVisualizedSpaceLocations;

        //Todo: This is assuming Quest 2 controller. Needs better abstraction to cover other types of controllers
        SingleActionData m_xButtonAction;
        SingleActionData m_yButtonAction;
        SingleActionData m_aButtonAction;
        SingleActionData m_bButtonAction;
        DualActionData m_joyStickXAction;
        DualActionData m_joyStickYAction;
    };
}
