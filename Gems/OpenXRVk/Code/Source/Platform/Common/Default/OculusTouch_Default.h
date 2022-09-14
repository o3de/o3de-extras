/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <OpenXRVk/InputDeviceXRController.h>

////////////////////////////////////////////////////////////////////////////////////////////////////
namespace AzFramework
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    //! Platform/API-specific implementation for Oculus Touch Controller input device
    class InputDeviceOculusTouch
        : public InputDeviceXRController::Implementation
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////////////
        // Allocator
        AZ_CLASS_ALLOCATOR(InputDeviceOculusTouch, AZ::SystemAllocator, 0);

        ////////////////////////////////////////////////////////////////////////////////////////////
        InputDeviceOculusTouch(InputDeviceXRController& inputDevice);
        ~InputDeviceOculusTouch() override;

    private:
        ////////////////////////////////////////////////////////////////////////////////////////////
        //! @see AzFramework::InputDeviceXRController::Implementation::IsConnected
        bool IsConnected() const override;

        ////////////////////////////////////////////////////////////////////////////////////////////
        //! @see AzFramework::InputDeviceXRController::Implementation::SetVibration
        void SetVibration(float leftMotorSpeedNormalized, float rightMotorSpeedNormalized) override;

        ////////////////////////////////////////////////////////////////////////////////////////////
        //! @see AzFramework::InputDeviceXRController::Implementation::TickInputDevice
        void TickInputDevice() override;

        ////////////////////////////////////////////////////////////////////////////////////////////
        //! @see AzFramework::InputDeviceXRController::Implementation::OnRawInputDeviceChangeEvent
        //void OnRawInputDeviceChangeEvent() override;

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Data
        RawXRControllerState m_rawControllerState;  //!< The latest raw xr controller input state
        bool m_isConnected{}; //!< Is the controller(s) currently connected?
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // static
    InputDeviceXRController::Implementation* InputDeviceXRController::Implementation::Create(
        InputDeviceXRController& inputDevice)
    {
        // Should check and make sure that OpenXR api is "available" here.  ??
        return aznew InputDeviceOculusTouch(inputDevice);
    }

} // namespace AzFramework
