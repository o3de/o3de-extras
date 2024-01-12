/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzFramework/Asset/AssetSystemBus.h>

//#include <Atom/RPI.Public/XR/XRSpaceNotificationBus.h>
#include <Atom/RPI.Reflect/Asset/AssetUtils.h>

//#include <OpenXRVk/OpenXRVkInstance.h>
//#include <OpenXRVk/OpenXRVkSession.h>
//#include <OpenXRVk/OpenXRVkDevice.h>
//#include <OpenXRVk/OpenXRVkSpace.h>
//#include <OpenXRVk/OpenXRVkUtils.h>
#include "OpenXRActionsBindingAsset.h"
#include <OpenXRVk/OpenXRInteractionProfileBus.h>
#include <OpenXRVk/OpenXRVkUtils.h>
#include "OpenXRActionsManager.h"

namespace OpenXRVk
{
    // XR::Ptr<Action> Action::Create()
    // {
    //     const auto newInput = aznew Action;
    //     //newInput->m_xrController.SetImplementation(&AzFramework::InputDeviceXRController::Implementation::Create);
    //     //newInput->m_xrControllerImpl = newInput->m_xrController.GetImplementation();
    //     return newInput;
    // }

    bool ActionsManager::Init(XrInstance xrInstance, XrSession xrSession)
    {
        m_xrInstance = xrInstance;
        m_xrSession = xrSession;

        // OpenXR only allows to define ActionSets during session creation.
        // From the point of view of O3DE, the developer defines action sets
        // in an asset of type OpenXRActionBindingsAsset.
        // The default source path for said asset is "@project@/openxr.xractions".
        const auto actionsBindingAsset = AZ::RPI::AssetUtils::LoadCriticalAsset<OpenXRActionBindingsAsset>({ DefaultActionsAssetPath });
        if (!actionsBindingAsset.IsReady())
        {
            AZ_Printf(LogName, "This application won't support user interactions. Default action bindings asset [%s] not found.\n", DefaultActionsAssetPath);
            return true;
        }

        AZStd::unordered_set<XrPath> activeProfiles;
        AZStd::vector<XrActionSuggestedBinding> activeBindings;
        for (const auto& actionSet : actionsBindingAsset->m_actionSets)
        {
            if (!InitActionSetInternal(actionSet, activeProfiles, activeBindings))
            {
                return false;
            }
        }

        if (activeBindings.empty() || activeProfiles.empty())
        {
            AZ_Printf(LogName, "This application will run without actions.\n");
            return true;
        }

        // Register the bindings for each active interaction profile.
        for (const auto& profilePath : activeProfiles)
        {
            XrInteractionProfileSuggestedBinding suggestedBindings{ XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING };
            suggestedBindings.interactionProfile = profilePath;
            suggestedBindings.suggestedBindings = activeBindings.data();
            suggestedBindings.countSuggestedBindings = static_cast<uint32_t>(activeBindings.size());
            XrResult result = xrSuggestInteractionProfileBindings(m_xrInstance, &suggestedBindings);
            WARN_IF_UNSUCCESSFUL(result);
        }

        AZStd::vector<XrActionSet> xrActionSets;
        xrActionSets.reserve(m_actionSets.size());
        size_t actionSetIdx = 0;
        for (const auto& actionSetInfo : m_actionSets)
        {
            xrActionSets.push_back(actionSetInfo.m_xrActionSet);
            m_activeActionSets.set(actionSetIdx, true); // By default all actionSets will be active.
            actionSetIdx++;
        }
        RecreateXrActiveActionSets();

        XrSessionActionSetsAttachInfo attachInfo{ XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO };
        attachInfo.countActionSets = static_cast<uint32_t>(xrActionSets.size());
        attachInfo.actionSets = xrActionSets.data();
        XrResult result = xrAttachSessionActionSets(m_xrSession, &attachInfo);
        if (IsError(result))
        {
            m_xrActiveActionSets.clear();
            PrintXrError(LogName, result, "Failed to attach %zu action sets to the session.", xrActionSets.size());
            return false;
        }

        return true;
    }

    bool ActionsManager::SyncActions()
    {
        if (m_xrActiveActionSets.empty())
        {
            // Nothing to do
            return true;
        }

        XrActionsSyncInfo syncInfo{ XR_TYPE_ACTIONS_SYNC_INFO };
        syncInfo.countActiveActionSets = aznumeric_cast<uint32_t>(m_xrActiveActionSets.size());
        syncInfo.activeActionSets = m_xrActiveActionSets.data();
        XrResult result = xrSyncActions(m_xrSession, &syncInfo);
        if (IsError(result))
        {
            PrintXrError(LogName, result, "Failed to sync %zu actionSets.\n", m_xrActiveActionSets.size());
            return false;
        }

        return true;
    }



    bool ActionsManager::InitActionSetInternal(const OpenXRActionSet& actionSet,
        AZStd::unordered_set<XrPath>& activeProfiles,
        AZStd::vector<XrActionSuggestedBinding>& activeBindings)
    {
        // Create an action set.
        XrActionSetCreateInfo actionSetCreateInfo{};
        actionSetCreateInfo.type = XR_TYPE_ACTION_SET_CREATE_INFO;
        azstrcpy(actionSetCreateInfo.actionSetName, sizeof(actionSetCreateInfo.actionSetName), actionSet.m_name.c_str());
        const char* localizedNameCStr = actionSet.m_name.c_str();
        if (!actionSet.m_localizedName.empty())
        {
            localizedNameCStr = actionSet.m_localizedName.c_str();
        }
        azstrcpy(actionSetCreateInfo.localizedActionSetName, sizeof(actionSetCreateInfo.localizedActionSetName), localizedNameCStr);
        actionSetCreateInfo.priority = actionSet.m_priority;

        {
            ActionSetInfo newActionSetInfo;
            newActionSetInfo.m_name = actionSet.m_name;
            XrResult result = xrCreateActionSet(m_xrInstance, &actionSetCreateInfo, &newActionSetInfo.m_xrActionSet);
            if (IsError(result))
            {
                PrintXrError(LogName, result, "Failed to instantiate actionSet named [%s].", actionSet.m_name.c_str());
                return false;
            }
            m_actionSets.emplace_back(AZStd::move(newActionSetInfo));
        }

        ActionSetInfo& newActionSetInfo = m_actionSets.back();
        for (const auto& action : actionSet.m_actions)
        {
            if (!InitActionBindingsInternal(newActionSetInfo, action, activeProfiles, activeBindings))
            {
                AZ_Error(LogName, false, "Failed to created action named [%s] under actionSet named [%s].",
                    action.m_name.c_str(), actionSet.m_name.c_str());
                return false;
            }
        }
        
        return true;
    }

    bool ActionsManager::InitActionBindingsInternal(ActionSetInfo& actionSetInfo, const OpenXRAction& action,
        AZStd::unordered_set<XrPath>& activeProfiles,
        AZStd::vector<XrActionSuggestedBinding>& activeBindings)
    {
        // One OpenXRAction object will become one XrAction.
        // An OpenXRAction contains a list of OpenXRActionPath that need to be bound.
        // The action type for each XrAction will be the same and it will be determined by
        // the action type of the first action in the list. 
        AZ_Assert(!action.m_actionPaths.empty(), "OpenXR Actions list must contain at least one action.");
        const auto& firstAction = action.m_actionPaths[0];

        auto interactionProviderIface = OpenXRInteractionProfileBus::FindFirstHandler(firstAction.m_interactionProfile);
        if (!interactionProviderIface)
        {
            AZ_Error(LogName, false, "Couldn't find interaction data provider with id [%s].", firstAction.m_interactionProfile.c_str())
            return false;
        }

        const auto firstActionInfo = interactionProviderIface->GetActionPathInfo(firstAction.m_userPath, firstAction.m_componentPath);

        XrActionCreateInfo actionCreateInfo{};
        actionCreateInfo.type = XR_TYPE_ACTION_CREATE_INFO;
        actionCreateInfo.actionType = firstActionInfo.m_actionType;
        azstrcpy(actionCreateInfo.actionName, sizeof(actionCreateInfo.actionName), action.m_name.c_str());
        const char* localizedNameCStr = action.m_name.c_str();
        if (!action.m_localizedName.empty())
        {
            localizedNameCStr = action.m_localizedName.c_str();
        }
        azstrcpy(actionCreateInfo.localizedActionName, sizeof(actionCreateInfo.localizedActionName), localizedNameCStr);
        actionCreateInfo.countSubactionPaths = 0; // Subactions are not supported.
        actionCreateInfo.subactionPaths = nullptr; // Subactions are not supported.

        XrAction newXrAction;
        XrResult result = xrCreateAction(actionSetInfo.m_xrActionSet, &actionCreateInfo, &newXrAction);
        if (IsError(result))
        {
            PrintXrError(LogName, result, "Failed to create action named %s.\n", action.m_name.c_str());
            return false;
        }

        // For each actionPath in the list, create the XrPath and its binding.
        uint32_t additionalBindingsCount = 0;
        for (const auto& actionPath : action.m_actionPaths)
        {
            interactionProviderIface = OpenXRInteractionProfileBus::FindFirstHandler(actionPath.m_interactionProfile);
            if (!interactionProviderIface)
            {
                AZ_Error(LogName, false, "Couldn't find interaction data provider with id [%s].", actionPath.m_interactionProfile.c_str())
                    return false;
            }

            const auto pathInfo = interactionProviderIface->GetActionPathInfo(actionPath.m_userPath, actionPath.m_componentPath);
            if (pathInfo.m_absolutePath.empty())
            {
                AZ_Warning(LogName, false, "Failed to retrieve action path info for profile [%s], user path [%s], component path [%s].\n",
                    actionPath.m_interactionProfile.c_str(), actionPath.m_userPath.c_str(), actionPath.m_componentPath.c_str());
                continue;
            }

            XrPath xrBindingPath;
            result = xrStringToPath(m_xrInstance, pathInfo.m_absolutePath.c_str(), &xrBindingPath);
            if (IsError(result))
            {
                PrintXrError(LogName, result, "Failed to create XrPath for action with profile [%s], absolute path [%s].\n",
                    actionPath.m_interactionProfile.c_str(), pathInfo.m_absolutePath.c_str());
                continue;
            }

            auto interactionProfilePathStr = interactionProviderIface->GetInteractionProviderPath();
            XrPath xrProviderPath;
            result = xrStringToPath(m_xrInstance, interactionProfilePathStr.c_str(), &xrProviderPath);
            if (IsError(result))
            {
                PrintXrError(LogName, result, "Failed to create XrPath for action provider [%s], provider path [%s].\n",
                    actionPath.m_interactionProfile.c_str(), interactionProfilePathStr.c_str());
                continue;
            }
            activeProfiles.emplace(xrProviderPath);

            XrActionSuggestedBinding binding;
            binding.action = newXrAction;
            binding.binding = xrBindingPath;
            activeBindings.push_back(binding);
            additionalBindingsCount++;
        }

        if (additionalBindingsCount < 1)
        {
            // This action has no bindings. Remove it.
            AZ_Warning(LogName, false, "The action [] had no bindings!.\n", action.m_name.c_str());
            xrDestroyAction(newXrAction);
            return true;
        }

        m_xrActions.push_back(newXrAction);
        uint16_t newActionIndex = aznumeric_cast<uint16_t>(m_xrActions.size() - 1);

        ActionInfo newActionInfo;
        newActionInfo.m_name = action.m_name;
        newActionInfo.m_actionType = firstActionInfo.m_actionType;
        newActionInfo.m_actionHandle = IOpenXRActions::ActionHandle(newActionIndex);
        actionSetInfo.m_actions.emplace(action.m_name, AZStd::move(newActionInfo));

        return true;
    }

    /////////////////////////////////////////////////
    /// OpenXRActionsInterface overrides
    AZStd::vector<AZStd::string> ActionsManager::GetAllActionSets() const
    {
        AZStd::vector<AZStd::string> retList;
        retList.reserve(m_actionSets.size());
        for (const auto& actionSetInfo : m_actionSets)
        {
            retList.push_back(actionSetInfo.m_name);
        }
        return retList;
    }


    AZStd::vector<AZStd::string> ActionsManager::GetActiveActionSets() const
    {
        AZStd::vector<AZStd::string> retList;
        retList.reserve(m_activeActionSets.count());
        for (size_t i = 0; i < m_actionSets.size(); i++)
        {
            if (m_activeActionSets[i])
            {
                retList.push_back(m_actionSets[i].m_name);
            }
        }
        return retList;
    }


    AZStd::vector<AZStd::string> ActionsManager::GetInactiveActionSets() const
    {
        AZStd::vector<AZStd::string> retList;
        retList.reserve(m_actionSets.size() - m_activeActionSets.count());
        for (size_t i = 0; i < m_actionSets.size(); i++)
        {
            if (!m_activeActionSets[i])
            {
                retList.push_back(m_actionSets[i].m_name);
            }
        }
        return retList;
    }


    AZ::Outcome<bool, AZStd::string> ActionsManager::ChangeActionSetState(const AZStd::string& actionSetName, bool activate)
    {
        constexpr bool recreateXrActiveActionSets = true;
        return ChangeActionSetStateInternal(actionSetName, activate, recreateXrActiveActionSets);
    }


    AZ::Outcome<bool, AZStd::string> ActionsManager::ChangeActionSetsState(const  AZStd::vector<AZStd::string>& actionSetNames, bool activate)
    {
        constexpr bool recreateXrActiveActionSets = false;
        for (const auto& actionSetName : actionSetNames)
        {
            auto outcome = ChangeActionSetStateInternal(actionSetName, activate, recreateXrActiveActionSets);
            if (!outcome.IsSuccess())
            {
                return outcome;
            }
        }
        RecreateXrActiveActionSets();
        return AZ::Success(true);
    }


    IOpenXRActions::ActionHandle ActionsManager::GetActionHandle(const AZStd::string& actionSetName, const AZStd::string& actionName) const
    {
        for (const auto& actionSetInfo : m_actionSets)
        {
            if (actionSetInfo.m_name != actionSetName)
            {
                continue;
            }
            const auto itor = actionSetInfo.m_actions.find(actionName);
            if (itor == actionSetInfo.m_actions.end())
            {
                return IOpenXRActions::ActionHandle::Null;
            }
            return itor->second.m_actionHandle;
        }
        return IOpenXRActions::ActionHandle::Null;
    }

    AZ::Outcome<bool, AZStd::string> ActionsManager::GetActionStateBoolean(ActionHandle actionHandle)
    {
        if (!actionHandle.IsValid())
        {
            return AZ::Failure("Invalid actionHandle!");
        }
        const auto actionIndex = actionHandle.GetIndex();

        XrActionStateBoolean state { XR_TYPE_ACTION_STATE_BOOLEAN };
        XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
        getInfo.action = m_xrActions[actionIndex];
        XrResult result = xrGetActionStateBoolean(m_xrSession, &getInfo, &state);
        if (IsError(result))
        {
            return AZ::Failure(AZStd::string(GetResultString(result)));
        }

        return AZ::Success(state.currentState);
    }

    AZ::Outcome<float, AZStd::string> ActionsManager::GetActionStateFloat(ActionHandle actionHandle)
    {
        if (!actionHandle.IsValid())
        {
            return AZ::Failure("Invalid actionHandle!");
        }
        const auto actionIndex = actionHandle.GetIndex();

        XrActionStateFloat state{ XR_TYPE_ACTION_STATE_FLOAT };
        XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
        getInfo.action = m_xrActions[actionIndex];
        XrResult result = xrGetActionStateFloat(m_xrSession, &getInfo, &state);
        if (IsError(result))
        {
            return AZ::Failure(AZStd::string(GetResultString(result)));
        }

        return AZ::Success(state.currentState);
    }

    AZ::Outcome<AZ::Vector2, AZStd::string> ActionsManager::GetActionStateVector2(ActionHandle actionHandle)
    {
        if (!actionHandle.IsValid())
        {
            return AZ::Failure("Invalid actionHandle!");
        }
        const auto actionIndex = actionHandle.GetIndex();

        XrActionStateVector2f state{ XR_TYPE_ACTION_STATE_VECTOR2F };
        XrActionStateGetInfo getInfo{ XR_TYPE_ACTION_STATE_GET_INFO };
        getInfo.action = m_xrActions[actionIndex];
        XrResult result = xrGetActionStateVector2f(m_xrSession, &getInfo, &state);
        if (IsError(result))
        {
            return AZ::Failure(AZStd::string(GetResultString(result)));
        }

        return AZ::Success(AZ::Vector2(state.currentState.x, state.currentState.y));
    }

    AZ::Outcome<AZ::Transform, AZStd::string> ActionsManager::GetActionStatePose(ActionHandle actionHandle)
    {
        //FIXME!
        if (!actionHandle.IsValid())
        {
            return AZ::Failure("Invalid actionHandle!");
        }
        [[maybe_unused]] const auto actionIndex = actionHandle.GetIndex();
        AZ_Assert(false, "FIXME!");
        return AZ::Success(AZ::Transform::CreateIdentity());
    }


    AZ::Outcome<bool, AZStd::string> ActionsManager::ApplyHapticVibrationAction(ActionHandle actionHandle,
        uint64_t durationNanos, float frequencyHz, float amplitude)
    {
        if (!actionHandle.IsValid())
        {
            return AZ::Failure("Invalid actionHandle!");
        }
        const auto actionIndex = actionHandle.GetIndex();

        // fire haptics using output action
        XrHapticVibration vibration{ XR_TYPE_HAPTIC_VIBRATION };
        vibration.amplitude = AZStd::clamp(amplitude, 0.0f, 1.0f);
        vibration.duration = durationNanos;
        vibration.frequency = frequencyHz;
        XrHapticActionInfo hapticActionInfo{ XR_TYPE_HAPTIC_ACTION_INFO };
        hapticActionInfo.action = m_xrActions[actionIndex];
        XrResult result = xrApplyHapticFeedback(m_xrSession, &hapticActionInfo, (const XrHapticBaseHeader*)&vibration);
        if (IsError(result))
        {
            return AZ::Failure(AZStd::string(GetResultString(result)));
        }
        return AZ::Success(true);
    }

    AZ::Outcome<bool, AZStd::string> ActionsManager::StopHapticVibrationAction(ActionHandle actionHandle)
    {
        if (!actionHandle.IsValid())
        {
            return AZ::Failure("Invalid actionHandle!");
        }
        const auto actionIndex = actionHandle.GetIndex();

        // fire haptics using output action
        XrHapticActionInfo hapticActionInfo{ XR_TYPE_HAPTIC_ACTION_INFO };
        hapticActionInfo.action = m_xrActions[actionIndex];
        XrResult result = xrStopHapticFeedback(m_xrSession, &hapticActionInfo);
        if (IsError(result))
        {
            return AZ::Failure(AZStd::string(GetResultString(result)));
        }
        return AZ::Success(true);
    }
    /// OpenXRActionsInterface overrides
    /////////////////////////////////////////////////
    AZ::Outcome<bool, AZStd::string> ActionsManager::ChangeActionSetStateInternal(const AZStd::string& actionSetName, bool activate, bool recreateXrActiveActionSets)
    {
        // First get the index.
        size_t foundIdx = 0;
        for (const auto& actionSetInfo : m_actionSets)
        {
            if (actionSetInfo.m_name == actionSetName)
            {
                break;
            }
            foundIdx++;
        }

        if (foundIdx >= m_actionSets.size())
        {
            return AZ::Failure(AZStd::string::format(
                "ActionSet with name [%s] not found.", actionSetName.c_str()));
        }

        m_activeActionSets.set(foundIdx, activate);

        if (recreateXrActiveActionSets)
        {
            RecreateXrActiveActionSets();
        }

        return AZ::Success(true);
    }

    void ActionsManager::RecreateXrActiveActionSets()
    {
        // Recreate and cache the XrActionActionSet list.
        m_xrActiveActionSets.clear();
        for (size_t i = 0; i < m_actionSets.size(); i++)
        {
            if (m_activeActionSets[i])
            {
                XrActiveActionSet activeActionSet{ m_actionSets[i].m_xrActionSet, XR_NULL_PATH };
                m_xrActiveActionSets.push_back(activeActionSet);
            }
        }
    }

    // 
    // 
    // void Action::CreateAllActions(const XrInstance& xrInstance)
    // {
    //     // Get the XrPath for the left and right hands - we will use them as subaction paths.
    //     const AZStd::string leftHandPath{ m_xrControllerImpl->GetLeftHandSubPath() };
    //     const AZStd::string rightHandPath{ m_xrControllerImpl->GetRightHandSubPath() };
    // 
    //     XrResult result = xrStringToPath(xrInstance, leftHandPath.data(), &m_handSubactionPath[static_cast<uint32_t>(XR::Side::Left)]);
    //     WARN_IF_UNSUCCESSFUL(result);
    //     result = xrStringToPath(xrInstance, rightHandPath.data(), &m_handSubactionPath[static_cast<uint32_t>(XR::Side::Right)]);
    //     WARN_IF_UNSUCCESSFUL(result);
    // 
    //     // Lambda to create an action and path, store them in m_xrActionPaths
    //     using namespace AzFramework;
    //     auto createXrAction = [this, &xrInstance](const InputChannelId& channelId, const XrActionType actionType)
    //     {
    //         m_xrActionIndices[channelId] = m_xrActionPaths.size();
    //         m_xrActionPaths.push_back({});
    // 
    //         CreateAction(m_xrActionPaths.back().action, actionType, channelId.GetName(), channelId.GetName(),
    //             aznumeric_cast<AZ::u32>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());
    // 
    //         const AZStd::string xrPathStr{ m_xrControllerImpl->GetInputChannelPath(channelId) };
    //         [[maybe_unused]] const XrResult pathResult = xrStringToPath(xrInstance, xrPathStr.data(), &m_xrActionPaths.back().binding);
    //         WARN_IF_UNSUCCESSFUL(pathResult);
    //     };
    // 
    //     for (const InputChannelId& channelId : InputDeviceXRController::Button::All)
    //     {
    //         createXrAction(channelId, XR_ACTION_TYPE_BOOLEAN_INPUT);
    //     }
    // 
    //     for (const InputChannelId& channelId : InputDeviceXRController::Trigger::All)
    //     {
    //         createXrAction(channelId, XR_ACTION_TYPE_FLOAT_INPUT);
    //     }
    // 
    //     for (const InputChannelId& channelId : InputDeviceXRController::ThumbStickAxis1D::All)
    //     {
    //         createXrAction(channelId, XR_ACTION_TYPE_FLOAT_INPUT);
    //     }
    // 
    //     for (const InputChannelId& channelId : InputDeviceXRController::ControllerPosePosition::All)
    //     {
    //         createXrAction(channelId, XR_ACTION_TYPE_POSE_INPUT);
    //     }
    // 
    //     for (const InputChannelId& channelId : InputDeviceXRController::ControllerPoseOrientation::All)
    //     {
    //         createXrAction(channelId, XR_ACTION_TYPE_POSE_INPUT); // is this correct?
    //     }
    // 
    //     m_xrControllerImpl->RegisterTickCallback([this](){ PollActions(); });
    // }
    // 
    // AZ::RHI::ResultCode Action::InitializeActionSpace(XrSession xrSession)
    // {
    //     XrActionSpaceCreateInfo actionSpaceInfo{};
    //     actionSpaceInfo.type = XR_TYPE_ACTION_SPACE_CREATE_INFO;
    //     actionSpaceInfo.action = GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::LPos);
    //     actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
    //     actionSpaceInfo.subactionPath = m_handSubactionPath[static_cast<uint32_t>(XR::Side::Left)];
    // 
    //     XrResult result = xrCreateActionSpace(xrSession, &actionSpaceInfo, &m_handSpace[static_cast<uint32_t>(XR::Side::Left)]);
    //     WARN_IF_UNSUCCESSFUL(result);
    //     RETURN_RESULTCODE_IF_UNSUCCESSFUL(ConvertResult(result));
    // 
    //     actionSpaceInfo.action = GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::RPos);
    //     actionSpaceInfo.subactionPath = m_handSubactionPath[static_cast<uint32_t>(XR::Side::Right)];
    // 
    //     result = xrCreateActionSpace(xrSession, &actionSpaceInfo, &m_handSpace[static_cast<uint32_t>(XR::Side::Right)]);
    //     WARN_IF_UNSUCCESSFUL(result);
    // 
    //     return ConvertResult(result);
    // }
    // 
    // AZ::RHI::ResultCode Action::InitializeActionSets(XrSession xrSession) const
    // {
    //     XrSessionActionSetsAttachInfo attachInfo{};
    //     attachInfo.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO;
    //     attachInfo.countActionSets = 1;
    //     attachInfo.actionSets = &m_actionSet;
    // 
    //     const XrResult result = xrAttachSessionActionSets(xrSession, &attachInfo);
    //     WARN_IF_UNSUCCESSFUL(result);
    // 
    //     return ConvertResult(result);
    // }
    // 
    // void Action::ShutdownInternal()
    // {
    //     if (m_actionSet != XR_NULL_HANDLE)
    //     {
    //         for (const auto hand : { XR::Side::Left, XR::Side::Right })
    //         {
    //             xrDestroySpace(m_handSpace[static_cast<AZ::u32>(hand)]);
    //         }
    //         xrDestroyActionSet(m_actionSet);
    //     }
    // 
    //     // Turn off the tick callback and reset the (non-owning) impl pointer back to null
    //     m_xrControllerImpl->RegisterTickCallback(nullptr);
    //     m_xrControllerImpl = nullptr;
    // }
    // 
    // XrAction Action::GetAction(const AzFramework::InputChannelId& channelId) const
    // {
    //     // this is a private function and only input channel ids that were used to
    //     // initialize structures in this class should be passed.
    // 
    //     // "at" will assert if the channelId is something unexpected for xr controller
    //     const auto index = m_xrActionIndices.at(channelId);
    //     return m_xrActionPaths[index].action;
    // }
    // 
    // void Action::PollActions()
    // {
    //     const auto session = static_cast<Session*>(GetDescriptor().m_session.get());
    //     XrSession xrSession = session->GetXrSession();
    //     m_handActive = { XR_FALSE, XR_FALSE };
    // 
    //     auto& rawControllerData = m_xrControllerImpl->GetRawState();
    // 
    //     // Might not need to reset if we're constantly refreshing all raw values.
    //     // In the future we may want to store off a couple ticks of data in a history
    //     // so that derivatives and edge detection can be computed.
    //     rawControllerData.Reset();
    // 
    //     // Sync actions
    //     const XrActiveActionSet activeActionSet{ m_actionSet, XR_NULL_PATH };
    //     XrActionsSyncInfo syncInfo{};
    //     syncInfo.type = XR_TYPE_ACTIONS_SYNC_INFO;
    //     syncInfo.countActiveActionSets = 1;
    //     syncInfo.activeActionSets = &activeActionSet;
    // 
    //     XrResult result = xrSyncActions(xrSession, &syncInfo);
    //     if (result != XR_SUCCESS)
    //     {
    //         // This will hit when the device gets put down / goes idle.
    //         // So to avoid spam, just return here.
    //         return;
    //     }
    // 
    //     using namespace AzFramework;
    //     using xrc = InputDeviceXRController;
    // 
    //     // Updating digital buttons is somewhat unique, because it compacts and combines them all to a u32 with bit masks...
    //     for (const auto& [channelId, bitMask] : rawControllerData.m_buttonIdsToBitMasks)
    //     {
    //         XrActionStateGetInfo getButtonInfo{};
    //         getButtonInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
    //         getButtonInfo.next = nullptr;
    //         getButtonInfo.action = GetAction(channelId);
    //         getButtonInfo.subactionPath = XR_NULL_PATH;
    // 
    //         XrActionStateBoolean buttonValue{};
    //         buttonValue.type = XR_TYPE_ACTION_STATE_BOOLEAN;
    // 
    //         result = xrGetActionStateBoolean(xrSession, &getButtonInfo, &buttonValue);
    //         WARN_IF_UNSUCCESSFUL(result);
    // 
    //         rawControllerData.m_digitalButtonStates |= (
    //             (buttonValue.isActive == XR_TRUE && buttonValue.currentState == XR_TRUE)
    //             ? bitMask
    //             : 0
    //         );
    //     }
    // 
    //     // lambda that obtains a float state from an action...
    //     auto getActionStateFloat = [&xrSession, this](const InputChannelId& channelId) -> float
    //     {
    //         XrActionStateGetInfo getAnalogInfo{};
    //         getAnalogInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
    //         getAnalogInfo.next = nullptr;
    //         getAnalogInfo.action = GetAction(channelId);
    //         getAnalogInfo.subactionPath = XR_NULL_PATH;
    // 
    //         XrActionStateFloat analogValue{};
    //         analogValue.type = XR_TYPE_ACTION_STATE_FLOAT;
    // 
    //         const XrResult result = xrGetActionStateFloat(xrSession, &getAnalogInfo, &analogValue);
    //         WARN_IF_UNSUCCESSFUL(result);
    // 
    //         if (analogValue.isActive == XR_TRUE)
    //         {
    //             return analogValue.currentState;
    //         }
    //         return 0.f;
    //     };
    // 
    //     // Update Analog values...
    //     rawControllerData.m_leftTriggerState = getActionStateFloat(xrc::Trigger::LTrigger);
    //     rawControllerData.m_rightTriggerState = getActionStateFloat(xrc::Trigger::RTrigger);
    //     rawControllerData.m_leftGripState = getActionStateFloat(xrc::Trigger::LGrip);
    //     rawControllerData.m_rightGripState = getActionStateFloat(xrc::Trigger::RGrip);
    //     rawControllerData.m_leftThumbStickXState = getActionStateFloat(xrc::ThumbStickAxis1D::LX);
    //     rawControllerData.m_leftThumbStickYState = getActionStateFloat(xrc::ThumbStickAxis1D::LY);
    //     rawControllerData.m_rightThumbStickXState = getActionStateFloat(xrc::ThumbStickAxis1D::RX);
    //     rawControllerData.m_rightThumbStickYState = getActionStateFloat(xrc::ThumbStickAxis1D::RY);
    // 
    //     // Scale the rendered hand by 1.0f (open) to 0.5f (fully squeezed).
    //     m_handScale[static_cast<AZ::u32>(XR::Side::Left)] = 1.f - 0.5f * rawControllerData.m_leftGripState;
    //     m_handScale[static_cast<AZ::u32>(XR::Side::Right)] = 1.f - 0.5f * rawControllerData.m_rightGripState;
    // 
    //     // lambda that outputs vibration amount to a particular side
    //     auto setHapticVibration = [this, &xrSession](AZ::u32 side, float amount)
    //     {
    //         if (amount > 0.f)
    //         {
    //             XrHapticVibration hapticVibration{};
    //             hapticVibration.type = XR_TYPE_HAPTIC_VIBRATION;
    //             hapticVibration.amplitude = amount;
    //             hapticVibration.duration = XR_MIN_HAPTIC_DURATION;
    //             hapticVibration.frequency = XR_FREQUENCY_UNSPECIFIED;
    // 
    //             XrHapticActionInfo hapticActionInfo{};
    //             hapticActionInfo.type = XR_TYPE_HAPTIC_ACTION_INFO;
    //             hapticActionInfo.action = m_hapticAction;
    //             hapticActionInfo.subactionPath = m_handSubactionPath[side];
    // 
    //             [[maybe_unused]] const XrResult result = xrApplyHapticFeedback(
    //                 xrSession, &hapticActionInfo, reinterpret_cast<XrHapticBaseHeader*>(&hapticVibration));
    //             WARN_IF_UNSUCCESSFUL(result);
    //         }
    //     };
    // 
    //     setHapticVibration(static_cast<AZ::u32>(XR::Side::Left), rawControllerData.m_leftMotorVibrationValue);
    //     setHapticVibration(static_cast<AZ::u32>(XR::Side::Right), rawControllerData.m_rightMotorVibrationValue);
    //     // after the vibration values have been used, reset them
    //     rawControllerData.m_leftMotorVibrationValue = 0.f;
    //     rawControllerData.m_rightMotorVibrationValue = 0.f;
    // 
    //     // Check if the Quit (Home) button was pressed this sync...
    //     const bool quitPressed = GetButtonState(InputDeviceXRController::Button::Home);
    //     if (quitPressed && !m_wasQuitPressedLastSync)
    //     {
    //         result = xrRequestExitSession(xrSession);
    //         WARN_IF_UNSUCCESSFUL(result);
    //     }
    //     m_wasQuitPressedLastSync = quitPressed;
    // }
    // 
    // 
    // bool Action::UpdateXrSpaceLocations(const OpenXRVk::Device& device, XrTime predictedDisplayTime, AZStd::vector<XrView>& xrViews)
    // {
    //     const auto thisDevice = static_cast<Device*>(GetDescriptor().m_device.get());
    //     if (thisDevice != &device)
    //     {
    //         return false;
    //     }
    // 
    //     auto& rawControllerData = m_xrControllerImpl->GetRawState();
    //     const auto session = static_cast<Session*>(GetDescriptor().m_session.get());
    //     XrSession xrSession = session->GetXrSession();
    //     XrSpace xrBaseSpaceForVisualization = session->GetXrSpace(session->GetBaseSpaceTypeForVisualization());
    //     XrSpace xrBaseSpaceForJoysticks = session->GetXrSpace(session->GetBaseSpaceTypeForControllers());
    // 
    //     // Update poses
    //     for (const auto hand : { XR::Side::Left, XR::Side::Right })
    //     {
    //         XrActionStateGetInfo getInfo{};
    //         getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
    //         getInfo.action = GetPoseAction(static_cast<AZ::u32>(hand));
    // 
    //         XrActionStatePose poseState{};
    //         poseState.type = XR_TYPE_ACTION_STATE_POSE;
    // 
    //         XrResult result = xrGetActionStatePose(xrSession, &getInfo, &poseState);
    //         WARN_IF_UNSUCCESSFUL(result);
    //         m_handActive[static_cast<AZ::u32>(hand)] = poseState.isActive;
    // 
    //         LocateControllerSpace(predictedDisplayTime, xrBaseSpaceForJoysticks, static_cast<AZ::u32>(hand));
    //     }
    // 
    //     // Cache 3d location information
    //     for (AZ::u32 i = 0; i < static_cast<AZ::u32>(SpaceType::Count); i++)
    //     {
    //         const auto spaceType = static_cast<SpaceType>(i);
    //         LocateVisualizedSpace(predictedDisplayTime, session->GetXrSpace(spaceType),
    //             xrBaseSpaceForVisualization, spaceType);
    //     }
    // 
    //     rawControllerData.m_leftPositionState = AzPositionFromXrPose(m_handSpaceLocation[static_cast<AZ::u32>(XR::Side::Left)].pose);
    //     rawControllerData.m_rightPositionState = AzPositionFromXrPose(m_handSpaceLocation[static_cast<AZ::u32>(XR::Side::Right)].pose);
    // 
    //     rawControllerData.m_leftOrientationState = AzQuaternionFromXrPose(m_handSpaceLocation[static_cast<AZ::u32>(XR::Side::Left)].pose);
    //     rawControllerData.m_rightOrientationState = AzQuaternionFromXrPose(m_handSpaceLocation[static_cast<AZ::u32>(XR::Side::Right)].pose);
    // 
    //     if (LocateEyeViews(predictedDisplayTime, xrViews))
    //     {
    //         //! Time to notify the engine that we have new poses.
    //         const auto& xrSpaceLocationHeadToBase = m_xrVisualizedSpaceLocations[OpenXRVk::SpaceType::View];
    //         const auto baseToHeadTm = AzTransformFromXrPose(xrSpaceLocationHeadToBase.pose);
    //         const auto headToLeftEyeTm = AzTransformFromXrPose(xrViews[0].pose);
    //         const auto headToRightEyeTm = AzTransformFromXrPose(xrViews[1].pose);
    // 
    //         AZ::RPI::XRSpaceNotificationBus::Broadcast(&AZ::RPI::XRSpaceNotifications::OnXRSpaceLocationsChanged,
    //             baseToHeadTm, headToLeftEyeTm, headToRightEyeTm);
    // 
    //         return true;
    //     }
    // 
    //     return false;
    // }
    // 
    // bool Action::LocateEyeViews(XrTime predictedDisplayTime, AZStd::vector<XrView>& xrViews)
    // {
    //     const auto session = static_cast<Session*>(GetDescriptor().m_session.get());
    //     XrSession xrSession = session->GetXrSession();
    //     const auto xrVkInstance = static_cast<Instance*>(GetDescriptor().m_instance.get());
    // 
    //     // Let's get the FOV data, which for most practical purposes it is always the same
    //     // across all frames. But most importantly we need to get the location of each Eye relative to the View Space pose.
    // 
    //     Space* xrSpace = static_cast<Space*>(session->GetSpace());
    // 
    //     XrViewState viewState{ XR_TYPE_VIEW_STATE };
    //     uint32_t viewCapacityInput = aznumeric_cast<uint32_t>(xrViews.size());
    //     uint32_t viewCountOutput = 0;
    // 
    //     XrViewLocateInfo viewLocateInfo{ XR_TYPE_VIEW_LOCATE_INFO };
    //     viewLocateInfo.viewConfigurationType = xrVkInstance->GetViewConfigType();
    //     viewLocateInfo.displayTime = predictedDisplayTime;
    //     viewLocateInfo.space = xrSpace->GetXrSpace(OpenXRVk::SpaceType::View);
    // 
    //     XrResult result = xrLocateViews(xrSession, &viewLocateInfo, &viewState, viewCapacityInput, &viewCountOutput, xrViews.data());
    //     ASSERT_IF_UNSUCCESSFUL(result);
    // 
    //     if ((viewState.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) == 0 ||
    //         (viewState.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0)
    //     {
    //         //There is no valid tracking poses for the views
    //         return false;
    //     }
    // 
    //     AZ_Error(LogName, viewCountOutput == viewCapacityInput, "Size mismatch between xrLocateViews %i and xrEnumerateViewConfigurationViews %i", viewCountOutput, viewCapacityInput);
    // 
    //     return (viewCountOutput == viewCapacityInput);
    // }
    // 
    // 
    // void Action::LocateControllerSpace(XrTime predictedDisplayTime, XrSpace baseSpace, AZ::u32 handIndex)
    // {
    //     XrSpaceLocation spaceLocation{};
    //     spaceLocation.type = XR_TYPE_SPACE_LOCATION;
    //     if (const XrResult result = xrLocateSpace(m_handSpace[handIndex], baseSpace, predictedDisplayTime, &spaceLocation);
    //         result == XR_SUCCESS)
    //     {
    //         if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
    //             (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0)
    //         {
    //             m_handSpaceLocation[handIndex] = spaceLocation;
    //         }
    //     }
    // }
    // 
    // void Action::LocateVisualizedSpace(XrTime predictedDisplayTime, XrSpace space, XrSpace baseSpace, OpenXRVk::SpaceType visualizedSpaceType)
    // {
    //     XrSpaceLocation spaceLocation{};
    //     spaceLocation.type = XR_TYPE_SPACE_LOCATION;
    //     if (const XrResult result = xrLocateSpace(space, baseSpace, predictedDisplayTime, &spaceLocation);
    //         result == XR_SUCCESS)
    //     {
    //         if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
    //             (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0)
    //         {
    //             m_xrVisualizedSpaceLocations[static_cast<uint32_t>(visualizedSpaceType)] = spaceLocation;
    //         }
    //     }
    // }
    // 
    // AZ::RHI::ResultCode Action::GetControllerPose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData, bool convertToO3de) const
    // {
    //     if (handIndex < AZStd::size(m_handSpaceLocation))
    //     {
    //         outPoseData.m_orientation = AzQuaternionFromXrPose(m_handSpaceLocation[handIndex].pose, convertToO3de);
    //         outPoseData.m_position = AzPositionFromXrPose(m_handSpaceLocation[handIndex].pose, convertToO3de);
    //         return AZ::RHI::ResultCode::Success;
    //     }
    //     return AZ::RHI::ResultCode::Fail;
    // }
    // 
    // AZ::RHI::ResultCode Action::GetControllerTransform(AZ::u32 handIndex, AZ::Transform& outTransform, bool convertToO3de) const
    // {
    //     if (handIndex < AZStd::size(m_handSpaceLocation))
    //     {
    //         outTransform = AzTransformFromXrPose(m_handSpaceLocation[handIndex].pose, convertToO3de);
    //         outTransform.SetUniformScale(m_handScale[handIndex]);
    //         return AZ::RHI::ResultCode::Success;
    //     }
    //     return AZ::RHI::ResultCode::Fail;
    // }
    // 
    // AZ::RHI::ResultCode Action::GetVisualizedSpacePose(OpenXRVk::SpaceType visualizedSpaceType, AZ::RPI::PoseData& outPoseData, bool convertToO3de) const
    // {
    //     const auto spaceIndex = static_cast<AZ::u32>(visualizedSpaceType);
    //     if (spaceIndex < AZStd::size(m_xrVisualizedSpaceLocations))
    //     {
    //         outPoseData.m_orientation = AzQuaternionFromXrPose(m_xrVisualizedSpaceLocations[spaceIndex].pose, convertToO3de);
    //         outPoseData.m_position = AzPositionFromXrPose(m_xrVisualizedSpaceLocations[spaceIndex].pose, convertToO3de);
    //         return AZ::RHI::ResultCode::Success;
    //     }
    //     return AZ::RHI::ResultCode::Fail;
    // }
    // 
    // AZ::RHI::ResultCode Action::GetVisualizedSpaceTransform(OpenXRVk::SpaceType visualizedSpaceType, AZ::Transform& outTransform, bool convertToO3de) const
    // {
    //     const auto spaceIndex = static_cast<AZ::u32>(visualizedSpaceType);
    //     if (spaceIndex < AZStd::size(m_xrVisualizedSpaceLocations))
    //     {
    //         outTransform = AzTransformFromXrPose(m_xrVisualizedSpaceLocations[spaceIndex].pose, convertToO3de);
    //         return AZ::RHI::ResultCode::Success;
    //     }
    //     return AZ::RHI::ResultCode::Fail;
    // }
    // 
    // float Action::GetControllerScale(AZ::u32 handIndex) const
    // {
    //     return m_handScale[handIndex];
    // }
    // 
    // XrAction Action::GetSqueezeAction(AZ::u32 handIndex) const
    // {
    //     return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
    //         ? GetAction(AzFramework::InputDeviceXRController::Trigger::LGrip)
    //         : GetAction(AzFramework::InputDeviceXRController::Trigger::RGrip);
    // }
    // 
    // XrAction Action::GetPoseAction(AZ::u32 handIndex) const
    // {
    //     return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
    //         ? GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::LPos)
    //         : GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::RPos);
    // }
    // 
    // XrAction Action::GetVibrationAction() const
    // {
    //     return m_hapticAction;
    // }
    // 
    // XrAction Action::GetQuitAction() const
    // {
    //     return GetAction(AzFramework::InputDeviceXRController::Button::Home);
    // }
    // 
    // bool Action::GetButtonState(const AzFramework::InputChannelId& channelId) const
    // {
    //     const auto& state = m_xrControllerImpl->GetRawState();
    //     return state.GetDigitalButtonState(channelId);
    // }
    // 
    // bool Action::GetXButtonState() const
    // {
    //     return GetButtonState(AzFramework::InputDeviceXRController::Button::X);
    // }
    // 
    // bool Action::GetYButtonState() const
    // {
    //     return GetButtonState(AzFramework::InputDeviceXRController::Button::Y);
    // }
    // 
    // bool Action::GetAButtonState() const
    // {
    //     return GetButtonState(AzFramework::InputDeviceXRController::Button::A);
    // }
    // 
    // bool Action::GetBButtonState() const
    // {
    //     return GetButtonState(AzFramework::InputDeviceXRController::Button::B);
    // }
    // 
    // float Action::GetXJoyStickState(AZ::u32 handIndex) const
    // {
    //     const auto& state = m_xrControllerImpl->GetRawState();
    //     return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
    //         ? state.m_leftThumbStickXState
    //         : state.m_rightThumbStickXState;
    // }
    // 
    // float Action::GetYJoyStickState(AZ::u32 handIndex) const
    // {
    //     const auto& state = m_xrControllerImpl->GetRawState();
    //     return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
    //         ? state.m_leftThumbStickYState
    //         : state.m_rightThumbStickYState;
    // }
    // 
    // float Action::GetSqueezeState(AZ::u32 handIndex) const
    // {
    //     const auto& state = m_xrControllerImpl->GetRawState();
    //     return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
    //         ? state.m_leftGripState
    //         : state.m_rightGripState;
    // }
    // 
    // float Action::GetTriggerState(AZ::u32 handIndex) const
    // {
    //     const auto& state = m_xrControllerImpl->GetRawState();
    //     return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
    //         ? state.m_leftTriggerState
    //         : state.m_rightTriggerState;
    // }

} // namespace OpenXRVk
