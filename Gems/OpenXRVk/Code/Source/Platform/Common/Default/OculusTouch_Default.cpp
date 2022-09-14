/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "OculusTouch_Default.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
namespace AzFramework
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //! Map of digital button ids keyed by their button bitmask
    const AZStd::unordered_map<AZ::u32, const InputChannelId*> GetDigitalButtonIdByBitMaskMap()
    {
        const AZStd::unordered_map<AZ::u32, const InputChannelId*> buttonMap =
        {
            // These may need to be mapped to something else entirely, like a XRPath map
            { (1 << 0), &InputDeviceXRController::Button::A },
            { (1 << 1), &InputDeviceXRController::Button::B },
            { (1 << 2), &InputDeviceXRController::Button::X },
            { (1 << 3), &InputDeviceXRController::Button::Y },
            { (1 << 4), &InputDeviceXRController::Button::Home },
            { (1 << 5), &InputDeviceXRController::Button::Menu },
            { (1 << 6), &InputDeviceXRController::Button::L3 },
            { (1 << 7), &InputDeviceXRController::Button::R3 },
            { (1 << 8), &InputDeviceXRController::Button::TA },
            { (1 << 9), &InputDeviceXRController::Button::TB },
            { (1 << 10), &InputDeviceXRController::Button::TX },
            { (1 << 11), &InputDeviceXRController::Button::TY },
            { (1 << 12), &InputDeviceXRController::Button::TLStick },
            { (1 << 13), &InputDeviceXRController::Button::TRStick },
            { (1 << 14), &InputDeviceXRController::Button::TLRest },
            { (1 << 15), &InputDeviceXRController::Button::TRRest },
            { (1 << 16), &InputDeviceXRController::Button::TLTrig },
            { (1 << 17), &InputDeviceXRController::Button::TRTrig },
            { (1 << 18), &InputDeviceXRController::Button::TLThumb },
            { (1 << 19), &InputDeviceXRController::Button::TRThumb },
            { (1 << 20), &InputDeviceXRController::Button::TLIndex },
            { (1 << 21), &InputDeviceXRController::Button::TRIndex },
        };
        return buttonMap;
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
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceOculusTouch::~InputDeviceOculusTouch() = default;

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
        // TBD ...
    }

} // namespace AzFramework
