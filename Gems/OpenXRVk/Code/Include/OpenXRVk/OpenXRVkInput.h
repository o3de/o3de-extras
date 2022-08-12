/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRInput.h>
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
        AZ::RPI::PoseData GetControllerPose(AZ::u32 viewIndex) const;

        //! Return scale for a controller attached to a view index
        float GetControllerScale(AZ::u32 viewIndex) const;

        //! Return Pose data for a tracked space type (i.e visualizedSpaceType)
        AZ::RPI::PoseData GetVisualizedSpacePose(OpenXRVk::SpaceType visualizedSpaceType) const;

        //! Get the Grab action
        XrAction GetGrabAction() const;

        //! Get the Pose action
        XrAction GetPoseAction() const;

        //! Get the Vibration action
        XrAction GetVibrationAction() const;

        //! Get the Quit action
        XrAction GetQuitAction() const;
    private:

        //! Create a XrAction
        void CreateAction(XrAction& action, XrActionType actionType,
                          const char* actionName, const char* localizedActionName,
                          uint32_t countSubactionPathCount, const XrPath* subActionPaths);

        //! Destroy native objects
        void ShutdownInternal() override;

        XrActionSet m_actionSet{ XR_NULL_HANDLE };
        XrAction m_grabAction{ XR_NULL_HANDLE };
        XrAction m_poseAction{ XR_NULL_HANDLE };
        XrAction m_vibrateAction{ XR_NULL_HANDLE };
        XrAction m_quitAction{ XR_NULL_HANDLE };
        AZStd::array<XrPath, 2> m_handSubactionPath;
        AZStd::array<XrSpace, 2> m_handSpace;
        AZStd::array<float, 2> m_handScale = { { 1.0f, 1.0f } };
        AZStd::array<XrBool32, 2> m_handActive;

        AZStd::array<XrSpaceLocation, 2> m_handSpaceLocation;
        AZStd::array<XrSpaceLocation, SpaceType::Count> m_xrVisualizedSpaceLocations;
    };
}
