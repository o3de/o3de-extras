/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

//#include <XR/XRInput.h>
//#include <OpenXRVk/InputDeviceXRController.h>
//#include <OpenXRVk/OpenXRVkSpace.h>
//#include <Atom/RPI.Public/XR/XRRenderingInterface.h>

#include <openxr/openxr.h>

#include <OpenXRVk/OpenXRActionsInterface.h>

#include "OpenXRActionsBindingAsset.h"

namespace OpenXRVk
{
    // Class that will help manage XrActionSet/XrAction.
    // All XrActionSet and XrActions will be created from an
    // asset created by the developer where all actions of interest
    // are instantiated.
    class ActionsManager final :
        public OpenXRActionsInterface::Registrar
    {
    public:
        AZ_CLASS_ALLOCATOR(ActionsManager, AZ::SystemAllocator);
        AZ_RTTI(ActionsManager, "{79E2B285-073B-4042-93ED-B92E6C819CA6}", IOpenXRActions);

        static constexpr char LogName[] = "OpenXRVkActionsManager";

        //! Initialize various actions and actionSets according to the
        //! "openxr.xractions" action bindings asset.
        bool Init(XrInstance xrInstance, XrSession xrSession);

        //! Called by the Session each tick.
        bool SyncActions(XrTime predictedDisplayTime);

        /////////////////////////////////////////////////
        /// OpenXRActionsInterface overrides
        AZStd::vector<AZStd::string> GetAllActionSets() const override;
        AZStd::vector<AZStd::string> GetActiveActionSets() const override;
        AZStd::vector<AZStd::string> GetInactiveActionSets() const override;
        AZ::Outcome<bool, AZStd::string> ChangeActionSetState(const AZStd::string& actionSetName, bool activate) override;
        AZ::Outcome<bool, AZStd::string> ChangeActionSetsState(const  AZStd::vector<AZStd::string>& actionSetNames, bool activate) override;

        ActionHandle GetActionHandle(const AZStd::string& actionSetName, const AZStd::string& actionName) const override;

        AZ::Outcome<bool, AZStd::string> GetActionStateBoolean(ActionHandle actionHandle) const override;
        AZ::Outcome<float, AZStd::string> GetActionStateFloat(ActionHandle actionHandle) const override;
        AZ::Outcome<AZ::Vector2, AZStd::string> GetActionStateVector2(ActionHandle actionHandle) const override;

        AZ::Outcome<bool, AZStd::string> SetBaseVisualizedSpaceForPoseActions(const AZStd::string& visualizedSpaceName) override;
        const AZStd::string& GetBaseVisualizedSpaceForPoseActions() const override;
        AZ::Outcome<AZ::Transform, AZStd::string> GetActionStatePose(ActionHandle actionHandle) const override;
        
        AZ::Outcome<PoseWithVelocities, AZStd::string> GetActionStatePoseWithVelocities(ActionHandle actionHandle) const override;

        AZ::Outcome<bool, AZStd::string> ApplyHapticVibrationAction(ActionHandle actionHandle, uint64_t durationNanos, float frequencyHz, float amplitude) override;
        AZ::Outcome<bool, AZStd::string> StopHapticVibrationAction(ActionHandle actionHandle) override;
        /// OpenXRActionsInterface overrides
        /////////////////////////////////////////////////

    private:

        // Asset Cache relative path
        static constexpr char DefaultActionsAssetPath[] = "openxr.xractions";


        bool InitActionSetInternal(const OpenXRActionSet& actionSet,
            AZStd::unordered_set<XrPath>& activeProfiles,
            AZStd::vector<XrActionSuggestedBinding>& activeBindings);

        struct ActionInfo
        {
            AZStd::string m_name;
            XrActionType m_actionType = XR_ACTION_TYPE_MAX_ENUM;
            XrAction m_xrAction = XR_NULL_HANDLE;
            // Only valid if m_actionType is XR_ACTION_TYPE_POSE_INPUT
            XrSpace m_xrSpace = XR_NULL_HANDLE;
        };

        struct ActionSetInfo
        {
            AZStd::string m_name;
            XrActionSet m_xrActionSet;
            // The key is the name of the action.
            // The value is the index in @m_actions;
            AZStd::unordered_map<AZStd::string, IOpenXRActions::ActionHandle> m_actions;
        };

        bool InitActionBindingsInternal(ActionSetInfo& actionSetInfo, const OpenXRAction& action,
                                        AZStd::unordered_set<XrPath>& activeProfiles,
                                        AZStd::vector<XrActionSuggestedBinding>& activeBindings);

        XrAction CreateXrActionAndXrSpace(const ActionSetInfo& actionSetInfo,
            const OpenXRAction& action, const XrActionType actionType, XrSpace& newXrActionSpace) const;

        uint32_t AppendActionBindings(const OpenXRAction& action, XrAction newXrAction,
            AZStd::unordered_set<XrPath>& activeProfiles,
            AZStd::vector<XrActionSuggestedBinding>& activeBindings) const;
        
        AZ::Outcome<bool, AZStd::string> ChangeActionSetStateInternal(const AZStd::string& actionSetName, bool activate, bool recreateXrActiveActionSets = false);
        void RecreateXrActiveActionSets();

        XrInstance m_xrInstance = XR_NULL_HANDLE;
        XrSession m_xrSession = XR_NULL_HANDLE;
        // Updated each time SyncActions is called.
        XrTime m_predictedDisplaytime;

        AZStd::string m_baseVisualizedSpaceName;
        XrSpace m_xrBaseVisualizedSpace = XR_NULL_HANDLE;

        //! Each actionSet in this list is guaranteed to contain at least one valid action.
        AZStd::vector<ActionSetInfo> m_actionSets;
        //! This is a flat list of all actions across all actionSets.
        //! An IOpenXRActions::ActionHandle is actually an index into this list.
        AZStd::vector<ActionInfo> m_actions;

        //! 32 action sets should be enough
        static constexpr uint32_t MaxActionSets = 32;
        // Each time ChangeActionSetState or ChangeActionSetsState is called
        // the following lists are updated:
        //! Each bit is an index in @m_actionsSets
        AZStd::bitset<MaxActionSets> m_activeActionSets;
        //! Here we cache the list of OpenXR native handles for active action sets.
        AZStd::vector<XrActiveActionSet> m_xrActiveActionSets;

    };
}
