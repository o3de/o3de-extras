/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "OculusTouch_Default.h"

#include <OpenXRVk_Platform.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
namespace AzFramework
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //! Map of digital button ids keyed by their button bitmask
    AZStd::unordered_map<AZ::u32, const InputChannelId*> GetDigitalButtonIdByBitMaskMap()
    {
        using xrc = InputDeviceXRController;
        return {
            // These may need to be mapped to something else entirely, like a XRPath map
            { (1 << 0), &xrc::Button::A },      // /user/hand/right/input/a/click
            { (1 << 1), &xrc::Button::B },      // /user/hand/right/input/b/click
            { (1 << 2), &xrc::Button::X },      // /user/hand/left/input/x/click
            { (1 << 3), &xrc::Button::Y },      // /user/hand/left/input/y/click
            { (1 << 4), &xrc::Button::Home },   // /user/hand/right/input/system/click
            { (1 << 5), &xrc::Button::Menu },   // /user/hand/left/input/menu/click
            { (1 << 6), &xrc::Button::L3 },     // /user/hand/left/input/thumbstick/click
            { (1 << 7), &xrc::Button::R3 },     // /user/hand/right/input/thumbstick/click
            { (1 << 8), &xrc::Button::TA },     // /user/hand/right/input/a/touch
            { (1 << 9), &xrc::Button::TB },     // /user/hand/right/input/b/touch
            { (1 << 10), &xrc::Button::TX },    // /user/hand/left/input/x/touch
            { (1 << 11), &xrc::Button::TY },    // /user/hand/left/input/y/touch
            { (1 << 12), &xrc::Button::TLStick },   // /user/hand/left/input/thumbstick/touch
            { (1 << 13), &xrc::Button::TRStick },   // /user/hand/right/input/thumbstick/touch
            { (1 << 14), &xrc::Button::TLRest },    // /user/hand/left/input/thumbrest/touch
            { (1 << 15), &xrc::Button::TRRest },    // /user/hand/right/input/thumbrest/touch
            { (1 << 16), &xrc::Button::TLTrig },    // /user/hand/left/input/trigger/touch
            { (1 << 17), &xrc::Button::TRTrig },    // /user/hand/right/input/trigger/touch
        };
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceOculusTouch::InputDeviceOculusTouch(InputDeviceXRController& inputDevice)
        : InputDeviceXRController::Implementation(inputDevice)
        , m_rawControllerState(GetDigitalButtonIdByBitMaskMap())
    {
        // These are guesses for now
        m_rawControllerState.m_triggerMaxValue = 1.f;
        m_rawControllerState.m_triggerDeadZoneValue = 0.1f;
        m_rawControllerState.m_thumbStickMaxValue = 1.f;
        m_rawControllerState.m_leftThumbStickDeadZoneValue = 0.1f;
        m_rawControllerState.m_rightThumbStickDeadZoneValue = 0.1f;

        using xrc = InputDeviceXRController;
        m_xrPathMap = {
            { &xrc::Button::A, "/user/hand/right/input/a/click" },
            { &xrc::Button::B, "/user/hand/right/input/b/click" },
            { &xrc::Button::X, "/user/hand/left/input/x/click" },
            { &xrc::Button::Y, "/user/hand/left/input/y/click" },
            { &xrc::Button::Home, "/user/hand/right/input/system/click" },
            { &xrc::Button::Menu, "/user/hand/left/input/menu/click" },
            { &xrc::Button::L3, "/user/hand/left/input/thumbstick/click" },
            { &xrc::Button::R3, "/user/hand/right/input/thumbstick/click" },
            { &xrc::Button::TA, "/user/hand/right/input/a/touch" },
            { &xrc::Button::TB, "/user/hand/right/input/b/touch" },
            { &xrc::Button::TX, "/user/hand/left/input/x/touch" },
            { &xrc::Button::TY, "/user/hand/left/input/y/touch" },
            { &xrc::Button::TLStick, "/user/hand/left/input/thumbstick/touch" },
            { &xrc::Button::TRStick, "/user/hand/right/input/thumbstick/touch" },
            { &xrc::Button::TLRest, "/user/hand/left/input/thumbrest/touch" },
            { &xrc::Button::TRRest, "/user/hand/right/input/thumbrest/touch" },
            { &xrc::Button::TLTrig, "/user/hand/left/input/trigger/touch" },
            { &xrc::Button::TRTrig, "/user/hand/right/input/trigger/touch" },
            { &xrc::Trigger::LTrigger, "/user/hand/left/input/trigger/value" },
            { &xrc::Trigger::RTrigger, "/user/hand/right/input/trigger/value" },
            { &xrc::Trigger::LGrip, "/user/hand/left/input/squeeze/value" },
            { &xrc::Trigger::RGrip, "/user/hand/right/input/squeeze/value" },
            { &xrc::ThumbStickAxis1D::LX, "/user/hand/left/input/thumbstick/x" },
            { &xrc::ThumbStickAxis1D::LY, "/user/hand/left/input/thumbstick/y" },
            { &xrc::ThumbStickAxis1D::RX, "/user/hand/right/input/thumbstick/x" },
            { &xrc::ThumbStickAxis1D::RY, "/user/hand/right/input/thumbstick/y" },
            { &xrc::ControllerPosePosition::LPos, "/user/hand/left/input/grip/pose" },
            { &xrc::ControllerPosePosition::RPos, "/user/hand/right/input/grip/pose" },
            { &xrc::ControllerPoseOrientation::LOrient, "/user/hand/left/input/aim/pose" },
            { &xrc::ControllerPoseOrientation::ROrient, "/user/hand/right/input/aim/pose" },
        };

        //m_testMap = {
        //    { xrc::Button::A, { "/user/hand/right/input/a/click", xrc::Button::A.GetName(), (1 << 0) } },
        //    { xrc::Button::B, { "/user/hand/right/input/b/click", xrc::Button::B.GetName(), (1 << 1) } },
        //    { xrc::Button::X, { "/user/hand/left/input/x/click", xrc::Button::X.GetName(), (1 << 2) } },
        //    { xrc::Button::Y, { "/user/hand/left/input/y/click", xrc::Button::Y.GetName(), (1 << 3) } },
        //    { xrc::Button::Home, { "/user/hand/right/input/system/click", xrc::Button::Home.GetName(), (1 << 4) } },
        //    { xrc::Button::Menu, { "/user/hand/left/input/menu/click", xrc::Button::Home.GetName(), (1 << 5) } },
        //    { xrc::Button::L3, { "/user/hand/left/input/thumbstick/click", xrc::Button::L3.GetName(), (1 << 6) } },
        //    { xrc::Button::R3, { "/user/hand/right/input/thumbstick/click", xrc::Button::R3.GetName(), (1 << 7) } },
        //    { xrc::Button::TA, { "/user/hand/right/input/a/touch", xrc::Button::TA.GetName(), (1 << 8) } },
        //    { xrc::Button::TB, { "/user/hand/right/input/b/touch", xrc::Button::TB.GetName(), (1 << 9) } },
        //    { xrc::Button::TX, { "/user/hand/left/input/x/touch", xrc::Button::TX.GetName(), (1 << 10) } },
        //    { xrc::Button::TY, { "/user/hand/left/input/y/touch", xrc::Button::TY.GetName(), (1 << 11) } },
        //    { xrc::Button::TLStick, { "/user/hand/left/input/thumbstick/touch", xrc::Button::TLStick.GetName(), (1 << 12) } },
        //    { xrc::Button::TRStick, { "/user/hand/right/input/thumbstick/touch", xrc::Button::TRStick.GetName(), (1 << 13) } },
        //    { xrc::Button::TLRest, { "/user/hand/left/input/thumbrest/touch", xrc::Button::TLRest.GetName(), (1 << 14) } },
        //    { xrc::Button::TRRest, { "/user/hand/right/input/thumbrest/touch", xrc::Button::TRRest.GetName(), (1 << 15) } },
        //    { xrc::Button::TLTrig, { "/user/hand/left/input/trigger/touch", xrc::Button::TLTrig.GetName(), (1 << 16) } },
        //    { xrc::Button::TRTrig, { "/user/hand/right/input/trigger/touch", xrc::Button::TRTrig.GetName(), (1 << 17) } },

        //    { xrc::Trigger::LTrigger, { "/user/hand/left/input/trigger/value", xrc::Trigger::LTrigger.GetName(), (1 << 18) } },
        //    { xrc::Trigger::RTrigger, { "/user/hand/right/input/trigger/value", xrc::Trigger::RTrigger.GetName(), (1 << 19) } },
        //    { xrc::Trigger::LGrip, { "/user/hand/left/input/squeeze/value", xrc::Trigger::LGrip.GetName(), (1 << 20) } },
        //    { xrc::Trigger::RGrip, { "/user/hand/right/input/squeeze/value", xrc::Trigger::RGrip.GetName(), (1 << 21) } },

        //    { xrc::ThumbStickAxis1D::LX, { "/user/hand/left/input/thumbstick/x", xrc::ThumbStickAxis1D::LX.GetName(), (1 << 22) } },
        //    { xrc::ThumbStickAxis1D::LY, { "/user/hand/left/input/thumbstick/y", xrc::ThumbStickAxis1D::LY.GetName(), (1 << 23) } },
        //    { xrc::ThumbStickAxis1D::RX, { "/user/hand/right/input/thumbstick/x", xrc::ThumbStickAxis1D::RX.GetName(), (1 << 24) } },
        //    { xrc::ThumbStickAxis1D::RY, { "/user/hand/right/input/thumbstick/y", xrc::ThumbStickAxis1D::RY.GetName(), (1 << 25) } },

        //    { xrc::ControllerPosePosition::LPos, { "/user/hand/left/input/grip/pose", xrc::ControllerPosePosition::LPos.GetName(), (1 << 26) } },
        //    { xrc::ControllerPosePosition::RPos, { "/user/hand/right/input/grip/pose", xrc::ControllerPosePosition::RPos.GetName(), (1 << 27) } },
        //    { xrc::ControllerPoseOrientation::LOrient, { "/user/hand/left/input/aim/pose", xrc::ControllerPoseOrientation::LOrient.GetName(), (1 << 28) } },
        //    { xrc::ControllerPoseOrientation::ROrient, { "/user/hand/right/input/aim/pose", xrc::ControllerPoseOrientation::ROrient.GetName(), (1 << 29) } },
        //};
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceOculusTouch::~InputDeviceOculusTouch() = default;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    AZStd::string_view InputDeviceOculusTouch::GetInputChannelPath(const InputChannelId& channelId) const
    {
        if (const auto it = m_xrPathMap.find(&channelId);
            it != m_xrPathMap.end())
        {
            return it->second;
        }
        //if (const auto it = m_testMap.find(channelId);
        //    it != m_testMap.end())
        //{
        //    return AZStd::get<0>(it->second);
        //}
        return {};
    }

    AZStd::string_view InputDeviceOculusTouch::GetInputDeviceProfilePath() const
    {
        return { "/interaction_profiles/oculus/touch_controller" };
    }

    AZStd::string_view InputDeviceOculusTouch::GetLeftHandSubPath() const
    {
        return { "/user/hand/left" };
    }

    AZStd::string_view InputDeviceOculusTouch::GetRightHandSubPath() const
    {
        return { "/user/hand/right" };
    }

    void InputDeviceOculusTouch::RegisterTickCallback(TickCallbackFn callbackFn)
    {
        m_tickCallback = callbackFn;
    }

    InputDeviceOculusTouch::RawXRControllerState& InputDeviceOculusTouch::GetRawState()
    {
        return m_rawControllerState;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool InputDeviceOculusTouch::IsConnected() const
    {
        return m_isConnected;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceOculusTouch::SetVibration(
        [[maybe_unused]] float leftMotorSpeedNormalized,
        [[maybe_unused]] float rightMotorSpeedNormalized)
    {
        if (m_isConnected)
        {
            // TBD ...
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceOculusTouch::TickInputDevice()
    {
        if (m_tickCallback)
        {
            m_tickCallback();

            ProcessRawControllerState(m_rawControllerState);
        }
    }

} // namespace AzFramework
