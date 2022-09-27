/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkInput.h>
#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkSession.h>
#include <OpenXRVk/OpenXRVkDevice.h>
#include <OpenXRVk/OpenXRVkSpace.h>
#include <OpenXRVk/OpenXRVkUtils.h>
#include <AzCore/Casting/numeric_cast.h>

#include <../Common/Default/OculusTouch_Default.h>

namespace OpenXRVk
{
    XR::Ptr<Input> Input::Create()
    {
        const auto newInput = aznew Input;
        newInput->m_xrController.SetImplementation(&AzFramework::InputDeviceXRController::Implementation::Create);
        newInput->m_xrControllerImpl = newInput->m_xrController.GetImplementation();
        return newInput;
    }

    AZ::RHI::ResultCode Input::InitInternal()
    {
        const auto xrVkInstance = dynamic_cast<Instance*>(GetDescriptor().m_instance.get());
        const XrInstance xrInstance = xrVkInstance->GetXRInstance();

        // Create an action set.
        CreateActionSet(xrInstance);
        //XrActionSetCreateInfo actionSetInfo{};
        //actionSetInfo.type = XR_TYPE_ACTION_SET_CREATE_INFO;
        //azstrcpy(actionSetInfo.actionSetName, sizeof(actionSetInfo.actionSetName), "gameplay");
        //azstrcpy(actionSetInfo.localizedActionSetName, sizeof(actionSetInfo.localizedActionSetName), "Gameplay");
        //actionSetInfo.priority = 0;
        //XrResult result = xrCreateActionSet(xrInstance, &actionSetInfo, &m_actionSet);
        //WARN_IF_UNSUCCESSFUL(result);

        CreateAllActions(xrInstance);

        // Get the XrPath for the left and right hands - we will use them as subaction paths.
        //result = xrStringToPath(xrInstance, "/user/hand/left", &m_handSubactionPath[static_cast<uint32_t>(XR::Side::Left)]);
        //WARN_IF_UNSUCCESSFUL(result);
        //result = xrStringToPath(xrInstance, "/user/hand/right", &m_handSubactionPath[static_cast<uint32_t>(XR::Side::Right)]);
        //WARN_IF_UNSUCCESSFUL(result);

        // Create actions.
        // Create an input action for grabbing objects with the left and right hands.
        //CreateAction(m_squeezeAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "squeeze_object", "Squeeze Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

        //CreateAction(m_triggerAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "trigger_object", "Trigger Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

        //CreateAction(m_poseAction, XR_ACTION_TYPE_POSE_INPUT, "hand_pose", "Hand Pose",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

        //CreateAction(m_vibrateAction, XR_ACTION_TYPE_VIBRATION_OUTPUT, "vibrate_hand", "Vibrate Hand",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

        //CreateAction(m_quitAction, XR_ACTION_TYPE_BOOLEAN_INPUT, "quit_session", "Quit Session", 0, nullptr);

        //CreateAction(m_xButtonAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "x_button", "X Button Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());
        //CreateAction(m_yButtonAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "y_button", "Y Button Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());
        //CreateAction(m_aButtonAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "a_button", "A Button Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());
        //CreateAction(m_bButtonAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "b_button", "B Button Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());
        //CreateAction(m_joyStickXAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "joystick_x", "JoyStick X Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());
        //CreateAction(m_joyStickYAction.m_actionHandle, XR_ACTION_TYPE_FLOAT_INPUT, "joystick_y", "JoyStick Y Object",
        //    aznumeric_cast<uint32_t>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

        //AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> squeezeValuePath{};
        //AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> triggerValuePath{};
        //AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> posePath{};
        //AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> hapticPath{};
        //AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> menuClickPath{};
        //AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> joyStickXPath{};
        //AZStd::array<XrPath, AZ::RPI::XRMaxNumControllers> joyStickYPath{};
        //XrPath xButtonValuePath;
        //XrPath yButtonValuePath;
        //XrPath aButtonValuePath;
        //XrPath bButtonValuePath;

        //result = xrStringToPath(xrInstance, "/user/hand/left/input/squeeze/value", &squeezeValuePath[static_cast<uint32_t>(XR::Side::Left)]);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/squeeze/value", &squeezeValuePath[static_cast<uint32_t>(XR::Side::Right)]);
        //result = xrStringToPath(xrInstance, "/user/hand/left/input/trigger/value", &triggerValuePath[static_cast<uint32_t>(XR::Side::Left)]);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/trigger/value", &triggerValuePath[static_cast<uint32_t>(XR::Side::Right)]);
        //result = xrStringToPath(xrInstance, "/user/hand/left/input/grip/pose", &posePath[static_cast<uint32_t>(XR::Side::Left)]);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/grip/pose", &posePath[static_cast<uint32_t>(XR::Side::Right)]);
        //result = xrStringToPath(xrInstance, "/user/hand/left/output/haptic", &hapticPath[static_cast<uint32_t>(XR::Side::Left)]);
        //result = xrStringToPath(xrInstance, "/user/hand/right/output/haptic", &hapticPath[static_cast<uint32_t>(XR::Side::Right)]);
        //result = xrStringToPath(xrInstance, "/user/hand/left/input/menu/click", &menuClickPath[static_cast<uint32_t>(XR::Side::Left)]);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/menu/click", &menuClickPath[static_cast<uint32_t>(XR::Side::Right)]);
        //result = xrStringToPath(xrInstance, "/user/hand/left/input/thumbstick/x", &joyStickXPath[static_cast<uint32_t>(XR::Side::Left)]);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/thumbstick/x", &joyStickXPath[static_cast<uint32_t>(XR::Side::Right)]);
        //result = xrStringToPath(xrInstance, "/user/hand/left/input/thumbstick/y", &joyStickYPath[static_cast<uint32_t>(XR::Side::Left)]);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/thumbstick/y", &joyStickYPath[static_cast<uint32_t>(XR::Side::Right)]);
        //result = xrStringToPath(xrInstance, "/user/hand/left/input/x/click", &xButtonValuePath);
        //result = xrStringToPath(xrInstance, "/user/hand/left/input/y/click", &yButtonValuePath);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/a/click", &aButtonValuePath);
        //result = xrStringToPath(xrInstance, "/user/hand/right/input/b/click", &bButtonValuePath);

        // Bindings for the Oculus Touch.
        XrPath oculusTouchInteractionProfilePath;
        AZStd::string controllerProfilePath{ m_xrControllerImpl->GetInputDeviceProfilePath() };
        [[maybe_unused]] XrResult result = xrStringToPath(xrInstance, controllerProfilePath.data(), &oculusTouchInteractionProfilePath);
        WARN_IF_UNSUCCESSFUL(result);
        //AZStd::vector<XrActionSuggestedBinding> bindings{
            //{ m_squeezeAction.m_actionHandle, squeezeValuePath[static_cast<uint32_t>(XR::Side::Left)] },
            //{ m_squeezeAction.m_actionHandle, squeezeValuePath[static_cast<uint32_t>(XR::Side::Right)] },
            //{ m_triggerAction.m_actionHandle, triggerValuePath[static_cast<uint32_t>(XR::Side::Left)] },
            //{ m_triggerAction.m_actionHandle, triggerValuePath[static_cast<uint32_t>(XR::Side::Right)] },
            //{ m_poseAction, posePath[static_cast<uint32_t>(XR::Side::Left)] },
            //{ m_poseAction, posePath[static_cast<uint32_t>(XR::Side::Right)] },
            //{ m_quitAction, menuClickPath[static_cast<uint32_t>(XR::Side::Left)] },
            //{ m_vibrateAction, hapticPath[static_cast<uint32_t>(XR::Side::Left)] },
            //{ m_vibrateAction, hapticPath[static_cast<uint32_t>(XR::Side::Right)] },
            //{ m_joyStickXAction.m_actionHandle, joyStickXPath[static_cast<uint32_t>(XR::Side::Left)] },
            //{ m_joyStickXAction.m_actionHandle, joyStickXPath[static_cast<uint32_t>(XR::Side::Right)] },
            //{ m_joyStickYAction.m_actionHandle, joyStickYPath[static_cast<uint32_t>(XR::Side::Left)] },
            //{ m_joyStickYAction.m_actionHandle, joyStickYPath[static_cast<uint32_t>(XR::Side::Right)] },
            //{ m_xButtonAction.m_actionHandle, xButtonValuePath },
            //{ m_yButtonAction.m_actionHandle, yButtonValuePath },
            //{ m_aButtonAction.m_actionHandle, aButtonValuePath },
            //{ m_bButtonAction.m_actionHandle, bButtonValuePath }
        //};

        XrInteractionProfileSuggestedBinding suggestedBindings{};
        suggestedBindings.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
        suggestedBindings.interactionProfile = oculusTouchInteractionProfilePath;
        suggestedBindings.suggestedBindings = m_xrActionPaths.data();
        suggestedBindings.countSuggestedBindings = aznumeric_cast<AZ::u32>(m_xrActionPaths.size());
        result = xrSuggestInteractionProfileBindings(xrInstance, &suggestedBindings);
        WARN_IF_UNSUCCESSFUL(result);

        //Init the location data so we dont read bad data when the device is in a bad state at start
        for (int i = 0; i < AZ::RPI::XRMaxNumControllers; i++)
        {
            m_handSpaceLocation[i].pose.orientation.x = 0.0f;
            m_handSpaceLocation[i].pose.orientation.y = 0.0f;
            m_handSpaceLocation[i].pose.orientation.z = 0.0f;
            m_handSpaceLocation[i].pose.orientation.w = 0.0f;
            m_handSpaceLocation[i].pose.position.x = 0.0f;
            m_handSpaceLocation[i].pose.position.y = 0.0f;
            m_handSpaceLocation[i].pose.position.z = 0.0f;
        }

        for (int i = 0; i < SpaceType::Count; i++)
        {
            m_xrVisualizedSpaceLocations[i].pose.orientation.x = 0.0f;
            m_xrVisualizedSpaceLocations[i].pose.orientation.y = 0.0f;
            m_xrVisualizedSpaceLocations[i].pose.orientation.z = 0.0f;
            m_xrVisualizedSpaceLocations[i].pose.orientation.w = 0.0f;
            m_xrVisualizedSpaceLocations[i].pose.position.x = 0.0f;
            m_xrVisualizedSpaceLocations[i].pose.position.y = 0.0f;
            m_xrVisualizedSpaceLocations[i].pose.position.z = 0.0f;
        }
        return ConvertResult(result);
    }

    void Input::CreateAction(XrAction& action, XrActionType actionType,
                             const char* actionName, const char* localizedActionName,
                             uint32_t countSubactionPathCount, const XrPath* subActionPaths) const
    {
        XrActionCreateInfo actionInfo{};
        actionInfo.type = XR_TYPE_ACTION_CREATE_INFO;
        actionInfo.actionType = actionType;
        azstrcpy(actionInfo.actionName, sizeof(actionInfo.actionName), actionName);
        azstrcpy(actionInfo.localizedActionName, sizeof(actionInfo.localizedActionName), localizedActionName);
        actionInfo.countSubactionPaths = countSubactionPathCount;
        actionInfo.subactionPaths = subActionPaths;
        [[maybe_unused]] const XrResult result = xrCreateAction(m_actionSet, &actionInfo, &action);
        WARN_IF_UNSUCCESSFUL(result);
    }

    void Input::CreateActionSet(const XrInstance& xrInstance)
    {
        // Create an action set.
        XrActionSetCreateInfo actionSetInfo{};
        actionSetInfo.type = XR_TYPE_ACTION_SET_CREATE_INFO;
        azstrcpy(actionSetInfo.actionSetName, sizeof(actionSetInfo.actionSetName), "gameplay");
        azstrcpy(actionSetInfo.localizedActionSetName, sizeof(actionSetInfo.localizedActionSetName), "Gameplay");
        actionSetInfo.priority = 0;
        [[maybe_unused]] const XrResult result = xrCreateActionSet(xrInstance, &actionSetInfo, &m_actionSet);
        WARN_IF_UNSUCCESSFUL(result);
    }

    void Input::CreateAllActions(const XrInstance& xrInstance)
    {
        // Get the XrPath for the left and right hands - we will use them as subaction paths.
        const AZStd::string leftHandPath{ m_xrControllerImpl->GetLeftHandSubPath() };
        const AZStd::string rightHandPath{ m_xrControllerImpl->GetRightHandSubPath() };
        XrResult result = xrStringToPath(xrInstance, leftHandPath.data(), &m_handSubactionPath[static_cast<uint32_t>(XR::Side::Left)]);
        WARN_IF_UNSUCCESSFUL(result);
        result = xrStringToPath(xrInstance, rightHandPath.data(), &m_handSubactionPath[static_cast<uint32_t>(XR::Side::Right)]);
        WARN_IF_UNSUCCESSFUL(result);

        using namespace AzFramework;
        for (const InputChannelId& channelId : InputDeviceXRController::Button::All)
        {
            m_xrActionIndices[&channelId] = m_xrActionPaths.size();
            m_xrActionPaths.push_back({});
            CreateAction(m_xrActionPaths.back().action, XR_ACTION_TYPE_BOOLEAN_INPUT, channelId.GetName(), channelId.GetName(),
                aznumeric_cast<AZ::u32>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

            const AZStd::string xrPathStr{ m_xrControllerImpl->GetInputChannelPath(channelId) };
            [[maybe_unused]] const XrResult pathResult = xrStringToPath(xrInstance, xrPathStr.data(), &m_xrActionPaths.back().binding);
            WARN_IF_UNSUCCESSFUL(pathResult);
        }

        for (const InputChannelId& channelId : InputDeviceXRController::Trigger::All)
        {
            m_xrActionIndices[&channelId] = m_xrActionPaths.size();
            m_xrActionPaths.push_back({});
            CreateAction(m_xrActionPaths.back().action, XR_ACTION_TYPE_FLOAT_INPUT, channelId.GetName(), channelId.GetName(),
                aznumeric_cast<AZ::u32>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

            const AZStd::string xrPathStr{ m_xrControllerImpl->GetInputChannelPath(channelId) };
            [[maybe_unused]] const XrResult pathResult = xrStringToPath(xrInstance, xrPathStr.data(), &m_xrActionPaths.back().binding);
            WARN_IF_UNSUCCESSFUL(pathResult);
        }

        for (const InputChannelId& channelId : InputDeviceXRController::ThumbStickAxis1D::All)
        {
            m_xrActionIndices[&channelId] = m_xrActionPaths.size();
            m_xrActionPaths.push_back({});
            CreateAction(m_xrActionPaths.back().action, XR_ACTION_TYPE_FLOAT_INPUT, channelId.GetName(), channelId.GetName(),
                aznumeric_cast<AZ::u32>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

            const AZStd::string xrPathStr{ m_xrControllerImpl->GetInputChannelPath(channelId) };
            [[maybe_unused]] const XrResult pathResult = xrStringToPath(xrInstance, xrPathStr.data(), &m_xrActionPaths.back().binding);
            WARN_IF_UNSUCCESSFUL(pathResult);
        }

        for (const InputChannelId& channelId : InputDeviceXRController::ControllerPosePosition::All)
        {
            m_xrActionIndices[&channelId] = m_xrActionPaths.size();
            m_xrActionPaths.push_back({});
            CreateAction(m_xrActionPaths.back().action, XR_ACTION_TYPE_POSE_INPUT, channelId.GetName(), channelId.GetName(),
                aznumeric_cast<AZ::u32>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

            const AZStd::string xrPathStr{ m_xrControllerImpl->GetInputChannelPath(channelId) };
            [[maybe_unused]] const XrResult pathResult = xrStringToPath(xrInstance, xrPathStr.data(), &m_xrActionPaths.back().binding);
            WARN_IF_UNSUCCESSFUL(pathResult);
        }

        for (const InputChannelId& channelId : InputDeviceXRController::ControllerPoseOrientation::All)
        {
            m_xrActionIndices[&channelId] = m_xrActionPaths.size();
            m_xrActionPaths.push_back({});
            CreateAction(m_xrActionPaths.back().action, XR_ACTION_TYPE_POSE_INPUT, channelId.GetName(), channelId.GetName(),
                aznumeric_cast<AZ::u32>(AZStd::size(m_handSubactionPath)), m_handSubactionPath.data());

            const AZStd::string xrPathStr{ m_xrControllerImpl->GetInputChannelPath(channelId) };
            [[maybe_unused]] const XrResult pathResult = xrStringToPath(xrInstance, xrPathStr.data(), &m_xrActionPaths.back().binding);
            WARN_IF_UNSUCCESSFUL(pathResult);
        }

        m_xrControllerImpl->RegisterTickCallback(AZStd::bind(&Input::PollActions, this));
    }

    AZ::RHI::ResultCode Input::InitializeActionSpace(XrSession xrSession)
    {
        XrActionSpaceCreateInfo actionSpaceInfo{};
        actionSpaceInfo.type = XR_TYPE_ACTION_SPACE_CREATE_INFO;
        //actionSpaceInfo.action = m_poseAction;
        actionSpaceInfo.action = GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::LPos);
        actionSpaceInfo.poseInActionSpace.orientation.w = 1.f;
        actionSpaceInfo.subactionPath = m_handSubactionPath[static_cast<uint32_t>(XR::Side::Left)];
        XrResult result = xrCreateActionSpace(xrSession, &actionSpaceInfo, &m_handSpace[static_cast<uint32_t>(XR::Side::Left)]);
        WARN_IF_UNSUCCESSFUL(result);
        RETURN_RESULTCODE_IF_UNSUCCESSFUL(ConvertResult(result));

        actionSpaceInfo.action = GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::RPos);
        actionSpaceInfo.subactionPath = m_handSubactionPath[static_cast<uint32_t>(XR::Side::Right)];
        result = xrCreateActionSpace(xrSession, &actionSpaceInfo, &m_handSpace[static_cast<uint32_t>(XR::Side::Right)]);
        WARN_IF_UNSUCCESSFUL(result);

        return ConvertResult(result);
    }

    AZ::RHI::ResultCode Input::InitializeActionSets(XrSession xrSession) const
    {
        XrSessionActionSetsAttachInfo attachInfo{};
        attachInfo.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO;
        attachInfo.countActionSets = 1;
        attachInfo.actionSets = &m_actionSet;
        const XrResult result = xrAttachSessionActionSets(xrSession, &attachInfo);
        WARN_IF_UNSUCCESSFUL(result);

        return ConvertResult(result);
    }

    void Input::ShutdownInternal()
    {
        if (m_actionSet != XR_NULL_HANDLE)
        {
            for (const auto hand : { XR::Side::Left, XR::Side::Right })
            {
                xrDestroySpace(m_handSpace[static_cast<AZ::u32>(hand)]);
            }
            xrDestroyActionSet(m_actionSet);
        }
    }

    XrAction Input::GetAction(const AzFramework::InputChannelId& channelId) const
    {
        // this is a private function and only input channel ids that were used to
        // initialize structures in this class should be used.

        // this will assert if the channelId is something unexpected for xr controller
        auto index = m_xrActionIndices.at(&channelId);
        return m_xrActionPaths[index].action;
    }

    void Input::PollActions()
    {
        const auto session = dynamic_cast<Session*>(GetDescriptor().m_session.get());
        XrSession xrSession = session->GetXrSession();
        const auto device = dynamic_cast<Device*>(GetDescriptor().m_device.get());
        m_handActive = { XR_FALSE, XR_FALSE };

        auto& rawControllerData = m_xrControllerImpl->GetRawState();
        // Might not need to reset if we're constantly refreshing all raw values.
        // In the future we may want to store off a couple ticks of data in a history
        // so that derivatives and edge detection can be computed.
        rawControllerData.Reset();

        // Sync actions
        const XrActiveActionSet activeActionSet{ m_actionSet, XR_NULL_PATH };
        XrActionsSyncInfo syncInfo{};
        syncInfo.type = XR_TYPE_ACTIONS_SYNC_INFO;
        syncInfo.countActiveActionSets = 1;
        syncInfo.activeActionSets = &activeActionSet;
        XrResult result = xrSyncActions(xrSession, &syncInfo);

        using namespace AzFramework;
        using xrc = InputDeviceXRController;

        // Updating digital buttons is unique, it compacts them all to a u32 with bit masks...
        for (const auto& [bitMask, channelId] : rawControllerData.m_digitalButtonIdsByBitMask)
        {
            XrActionStateGetInfo getButtonInfo{};
            getButtonInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
            getButtonInfo.next = nullptr;
            getButtonInfo.action = GetAction(*channelId);
            getButtonInfo.subactionPath = XR_NULL_PATH;

            XrActionStateBoolean buttonValue{};
            buttonValue.type = XR_TYPE_ACTION_STATE_BOOLEAN;

            result = xrGetActionStateBoolean(xrSession, &getButtonInfo, &buttonValue);
            WARN_IF_UNSUCCESSFUL(result);

            rawControllerData.m_digitalButtonStates |= (
                (buttonValue.isActive == XR_TRUE && buttonValue.currentState == XR_TRUE)
                ? bitMask
                : 0
            );
        }

        // lambda that obtains a float from an action...
        auto getActionStateFloat = [&xrSession, this](const InputChannelId& channelId) -> float
        {
            XrActionStateGetInfo getAnalogInfo{};
            getAnalogInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
            getAnalogInfo.next = nullptr;
            getAnalogInfo.action = GetAction(channelId);
            getAnalogInfo.subactionPath = XR_NULL_PATH;

            XrActionStateFloat analogValue{};
            analogValue.type = XR_TYPE_ACTION_STATE_FLOAT;

            const XrResult result = xrGetActionStateFloat(xrSession, &getAnalogInfo, &analogValue);
            WARN_IF_UNSUCCESSFUL(result);

            if (analogValue.isActive == XR_TRUE)
            {
                return analogValue.currentState;
            }
            return 0.f;
        };

        // Analog values...
        rawControllerData.m_leftTriggerState = getActionStateFloat(xrc::Trigger::LTrigger);
        rawControllerData.m_rightTriggerState = getActionStateFloat(xrc::Trigger::RTrigger);
        rawControllerData.m_leftGripState = getActionStateFloat(xrc::Trigger::LGrip);
        rawControllerData.m_rightGripState = getActionStateFloat(xrc::Trigger::RGrip);
        rawControllerData.m_leftThumbStickXState = getActionStateFloat(xrc::ThumbStickAxis1D::LX);
        rawControllerData.m_leftThumbStickYState = getActionStateFloat(xrc::ThumbStickAxis1D::LY);
        rawControllerData.m_rightThumbStickXState = getActionStateFloat(xrc::ThumbStickAxis1D::RX);
        rawControllerData.m_rightThumbStickYState = getActionStateFloat(xrc::ThumbStickAxis1D::RY);

        // Position and Orientation


        // Get pose and grab action state and start haptic vibrate when hand is 90% squeezed for testing purposes
        for (auto hand : { XR::Side::Left, XR::Side::Right })
        {
            //if (const bool isActive = UpdateActionState(xrSession, m_squeezeAction, static_cast<uint16_t>(hand));
            //    isActive)
            //{
            //    // Scale the rendered hand by 1.0f (open) to 0.5f (fully squeezed).
            //    m_handScale[static_cast<uint32_t>(hand)] = 1.0f - 0.5f * m_squeezeAction.m_actionState[static_cast<uint32_t>(hand)];
            //    if (m_squeezeAction.m_actionState[static_cast<uint32_t>(hand)] > 0.9f)
            //    {
            //        //This vibration event is currently added here for testing purposes.
            //        //Remove this when this is moved to an event that is triggered externally
            //        XrHapticVibration vibration{};
            //        vibration.type = XR_TYPE_HAPTIC_VIBRATION;
            //        vibration.amplitude = 0.5;
            //        vibration.duration = XR_MIN_HAPTIC_DURATION;
            //        vibration.frequency = XR_FREQUENCY_UNSPECIFIED;

            //        XrHapticActionInfo hapticActionInfo{};
            //        hapticActionInfo.type = XR_TYPE_HAPTIC_ACTION_INFO;
            //        hapticActionInfo.action = m_vibrateAction;
            //        hapticActionInfo.subactionPath = m_handSubactionPath[static_cast<uint32_t>(hand)];
            //        result = xrApplyHapticFeedback(xrSession, &hapticActionInfo, reinterpret_cast<XrHapticBaseHeader*>(&vibration));
            //        WARN_IF_UNSUCCESSFUL(result);
            //    }
            //}

            XrActionStateGetInfo getInfo{};
            getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
            //getInfo.action = m_poseAction;
            getInfo.action = GetPoseAction(static_cast<AZ::u32>(hand));
            XrActionStatePose poseState{};
            poseState.type = XR_TYPE_ACTION_STATE_POSE;
            result = xrGetActionStatePose(xrSession, &getInfo, &poseState);
            WARN_IF_UNSUCCESSFUL(result);
            m_handActive[static_cast<uint32_t>(hand)] = poseState.isActive;

            LocateControllerSpace(device->GetPredictedDisplayTime(), session->GetXrSpace(OpenXRVk::SpaceType::View), static_cast<uint32_t>(hand));

            //UpdateActionState(xrSession, m_triggerAction, static_cast<uint16_t>(hand));
            //UpdateActionState(xrSession, m_joyStickXAction, static_cast<uint16_t>(hand));
            //UpdateActionState(xrSession, m_joyStickYAction, static_cast<uint16_t>(hand));
        }

        //UpdateActionState(xrSession, m_xButtonAction, static_cast<uint32_t>(XR::Side::Left));
        //UpdateActionState(xrSession, m_yButtonAction, static_cast<uint32_t>(XR::Side::Left));
        //UpdateActionState(xrSession, m_aButtonAction, static_cast<uint32_t>(XR::Side::Right));
        //UpdateActionState(xrSession, m_bButtonAction, static_cast<uint32_t>(XR::Side::Right));

        //Cache 3d location information
        for (AZ::u32 i = 0; i < static_cast<AZ::u32>(SpaceType::Count); i++)
        {
            const auto spaceType = static_cast<SpaceType>(i);
            LocateVisualizedSpace(device->GetPredictedDisplayTime(), session->GetXrSpace(spaceType),
                                  session->GetXrSpace(OpenXRVk::SpaceType::View), spaceType);
        }

        // There were no subaction paths specified for the quit action, because we don't care which hand did it.
        XrActionStateGetInfo getInfo{};
        getInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
        getInfo.next = nullptr;
        getInfo.action = GetQuitAction();
        getInfo.subactionPath = XR_NULL_PATH;
        XrActionStateBoolean quitValue{};
        quitValue.type = XR_TYPE_ACTION_STATE_BOOLEAN;
        result = xrGetActionStateBoolean(xrSession, &getInfo, &quitValue);
        WARN_IF_UNSUCCESSFUL(result);
        if ((quitValue.isActive == XR_TRUE) && (quitValue.changedSinceLastSync == XR_TRUE) && (quitValue.currentState == XR_TRUE))
        {
            result = xrRequestExitSession(xrSession);
            WARN_IF_UNSUCCESSFUL(result);
        }
    }

    //bool Input::GetActionState(XrSession xrSession, XrAction xrAction, uint16_t handIndex, float& outputSate)
    //{
    //    XrActionStateGetInfo buttonGetInfo{};
    //    buttonGetInfo.type = XR_TYPE_ACTION_STATE_GET_INFO;
    //    buttonGetInfo.action = xrAction;
    //    buttonGetInfo.subactionPath = m_handSubactionPath[handIndex];

    //    XrActionStateFloat buttonValue{};
    //    buttonValue.type = XR_TYPE_ACTION_STATE_FLOAT;
    //    const XrResult result = xrGetActionStateFloat(xrSession, &buttonGetInfo, &buttonValue);
    //    WARN_IF_UNSUCCESSFUL(result);
    //    if (buttonValue.isActive == XR_TRUE)
    //    {
    //        outputSate = buttonValue.currentState;
    //        return true;
    //    }
    //    return false;
    //}

    //bool Input::UpdateActionState(XrSession xrSession, SingleActionData& actionData, uint16_t handIndex)
    //{
    //    return GetActionState(xrSession, actionData.m_actionHandle, handIndex, actionData.m_actionState);
    //}

    //bool Input::UpdateActionState(XrSession xrSession, DualActionData& actionData, uint16_t handIndex)
    //{
    //    return GetActionState(xrSession, actionData.m_actionHandle, handIndex, actionData.m_actionState[handIndex]);
    //}


    void Input::LocateControllerSpace(XrTime predictedDisplayTime, XrSpace baseSpace, AZ::u32 handIndex)
    {
        XrSpaceLocation spaceLocation{};
        spaceLocation.type = XR_TYPE_SPACE_LOCATION;
        if (const XrResult result = xrLocateSpace(m_handSpace[handIndex], baseSpace, predictedDisplayTime, &spaceLocation);
            result == XR_SUCCESS)
        {
            if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0)
            {
                m_handSpaceLocation[handIndex] = spaceLocation;
            }
        }
    }

    void Input::LocateVisualizedSpace(XrTime predictedDisplayTime, XrSpace space, XrSpace baseSpace, OpenXRVk::SpaceType visualizedSpaceType)
    {
        XrSpaceLocation spaceLocation{};
        spaceLocation.type = XR_TYPE_SPACE_LOCATION;
        if (const XrResult result = xrLocateSpace(space, baseSpace, predictedDisplayTime, &spaceLocation);
            result == XR_SUCCESS)
        {
            if ((spaceLocation.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                (spaceLocation.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0)
            {
                m_xrVisualizedSpaceLocations[static_cast<uint32_t>(visualizedSpaceType)] = spaceLocation;
            }
        }
    }

    AZ::RHI::ResultCode Input::GetControllerPose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const
    {
        if (handIndex < AZStd::size(m_handSpaceLocation))
        {
            const XrQuaternionf& orientation = m_handSpaceLocation[handIndex].pose.orientation;
            const XrVector3f& position = m_handSpaceLocation[handIndex].pose.position;
            outPoseData.m_orientation.Set(orientation.x, orientation.y, orientation.z, orientation.w);
            outPoseData.m_position.Set(position.x, position.y, position.z);
            return AZ::RHI::ResultCode::Success;
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode Input::GetVisualizedSpacePose(OpenXRVk::SpaceType visualizedSpaceType, AZ::RPI::PoseData& outPoseData) const
    {
        const auto spaceIndex = static_cast<uint32_t>(visualizedSpaceType);
        if (spaceIndex < AZStd::size(m_xrVisualizedSpaceLocations))
        {
            const XrQuaternionf& orientation = m_xrVisualizedSpaceLocations[spaceIndex].pose.orientation;
            const XrVector3f& position = m_xrVisualizedSpaceLocations[spaceIndex].pose.position;
            outPoseData.m_orientation.Set(orientation.x, orientation.y, orientation.z, orientation.w);
            outPoseData.m_position.Set(position.x, position.y, position.z);
            return AZ::RHI::ResultCode::Success;
        }
        return AZ::RHI::ResultCode::Fail;
    }

    float Input::GetControllerScale(AZ::u32 viewIndex) const
    {
        return m_handScale[viewIndex];
    }

    XrAction Input::GetSqueezeAction(AZ::u32 handIndex) const
    {
        return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
            ? GetAction(AzFramework::InputDeviceXRController::Trigger::LGrip)
            : GetAction(AzFramework::InputDeviceXRController::Trigger::RGrip);
        //return m_squeezeAction.m_actionHandle;
    }

    XrAction Input::GetPoseAction(AZ::u32 handIndex) const
    {
        return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
            ? GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::LPos)
            : GetAction(AzFramework::InputDeviceXRController::ControllerPosePosition::RPos);
        //return m_poseAction;
    }

    XrAction Input::GetVibrationAction() const
    {
        return {};  // TBD ...
        //return m_vibrateAction;
    }

    XrAction Input::GetQuitAction() const
    {
        return GetAction(AzFramework::InputDeviceXRController::Button::Home);
        //return m_quitAction;
    }

    float Input::GetXButtonState() const
    {
        return 0.f;
        //return m_xrControllerImpl->GetRawState().m_digitalButtonStates & ???
        //return m_xButtonAction.m_actionState;
    }

    float Input::GetYButtonState() const
    {
        return 0.f;
        //return m_yButtonAction.m_actionState;
    }

    float Input::GetAButtonState() const
    {
        return 0.f;
        //return m_aButtonAction.m_actionState;
    }

    float Input::GetBButtonState() const
    {
        return 0.f;
        //return m_bButtonAction.m_actionState;
    }

    float Input::GetXJoyStickState(AZ::u32 handIndex) const
    {
        const auto& state = m_xrControllerImpl->GetRawState();
        return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
            ? state.m_leftThumbStickXState
            : state.m_rightThumbStickXState;
        //return m_joyStickXAction.m_actionState[handIndex];
    }

    float Input::GetYJoyStickState(AZ::u32 handIndex) const
    {
        const auto& state = m_xrControllerImpl->GetRawState();
        return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
            ? state.m_leftThumbStickYState
            : state.m_rightThumbStickYState;
        //return m_joyStickYAction.m_actionState[handIndex];
    }

    float Input::GetSqueezeState(AZ::u32 handIndex) const
    {
        const auto& state = m_xrControllerImpl->GetRawState();
        return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
            ? state.m_leftGripState
            : state.m_rightGripState;
        //return m_squeezeAction.m_actionState[handIndex];
    }

    float Input::GetTriggerState(AZ::u32 handIndex) const
    {
        const auto& state = m_xrControllerImpl->GetRawState();
        return (handIndex == static_cast<AZ::u32>(XR::Side::Left))
            ? state.m_leftTriggerState
            : state.m_rightTriggerState;
        //return m_triggerAction.m_actionState[handIndex];
    }
}
