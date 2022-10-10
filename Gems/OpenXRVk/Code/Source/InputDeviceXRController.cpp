/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/InputDeviceXRController.h>

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzFramework/Input/Utils/AdjustAnalogInputForDeadZone.h>


namespace AzFramework
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // static
    bool InputDeviceXRController::IsXRControllerDevice(const InputDeviceId& inputDeviceId)
    {
        // Only need to check the name (crc) to check the device is an xr controller type.
        return (inputDeviceId.GetNameCrc32() == IdForIndex0.GetNameCrc32());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::Reflect(AZ::ReflectContext* context)
    {
        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
#define BEHAVIOR_XR_CONSTANT(channel)       Constant(channel.GetName(), BehaviorConstant(channel.GetName()))

            behaviorContext->Class<InputDeviceXRController>()
                ->Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::RuntimeOwn)
                ->Constant("name", BehaviorConstant(IdForIndex0.GetName()))
                // Standard digital buttons...
                ->BEHAVIOR_XR_CONSTANT(Button::A)
                ->BEHAVIOR_XR_CONSTANT(Button::B)
                ->BEHAVIOR_XR_CONSTANT(Button::X)
                ->BEHAVIOR_XR_CONSTANT(Button::Y)
                ->BEHAVIOR_XR_CONSTANT(Button::Home)
                ->BEHAVIOR_XR_CONSTANT(Button::Menu)
                ->BEHAVIOR_XR_CONSTANT(Button::L3)
                ->BEHAVIOR_XR_CONSTANT(Button::R3)
                // Touch capacitive...
                ->BEHAVIOR_XR_CONSTANT(Button::TA)
                ->BEHAVIOR_XR_CONSTANT(Button::TB)
                ->BEHAVIOR_XR_CONSTANT(Button::TX)
                ->BEHAVIOR_XR_CONSTANT(Button::TY)
                ->BEHAVIOR_XR_CONSTANT(Button::TLStick)
                ->BEHAVIOR_XR_CONSTANT(Button::TRStick)
                ->BEHAVIOR_XR_CONSTANT(Button::TLRest)
                ->BEHAVIOR_XR_CONSTANT(Button::TRRest)
                ->BEHAVIOR_XR_CONSTANT(Button::TLTrig)
                ->BEHAVIOR_XR_CONSTANT(Button::TRTrig)
                // Analog triggers...
                ->BEHAVIOR_XR_CONSTANT(Trigger::LTrigger)
                ->BEHAVIOR_XR_CONSTANT(Trigger::RTrigger)
                ->BEHAVIOR_XR_CONSTANT(Trigger::LGrip)
                ->BEHAVIOR_XR_CONSTANT(Trigger::RGrip)
                // Thumbsticks (1D)...
                ->BEHAVIOR_XR_CONSTANT(ThumbStickAxis1D::LX)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickAxis1D::LY)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickAxis1D::RX)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickAxis1D::RY)
                // Thumbsticks (2D)...
                ->BEHAVIOR_XR_CONSTANT(ThumbStickAxis2D::L)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickAxis2D::R)
                // Thumbstick directions...
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::LU)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::LD)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::LL)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::LR)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::RU)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::RD)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::RL)
                ->BEHAVIOR_XR_CONSTANT(ThumbStickDirection::RR)
                // Position (3D)...
                ->BEHAVIOR_XR_CONSTANT(ControllerPosePosition::LPos)
                ->BEHAVIOR_XR_CONSTANT(ControllerPosePosition::RPos)
                // Orientation (quaternion)...
                ->BEHAVIOR_XR_CONSTANT(ControllerPoseOrientation::LOrient)
                ->BEHAVIOR_XR_CONSTANT(ControllerPoseOrientation::ROrient)
            ;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //! Default constructor
    //! Using the default constructor will not create an implementation.  It is then up to the user
    //! to call InputDeviceXRController::SetImplementation and supply either a unique_ptr<Implementation>
    //! or an ImplementationFactory function.
    InputDeviceXRController::InputDeviceXRController()
        : InputDeviceXRController(InputDeviceId(Name, 0), nullptr)
    {
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceXRController::InputDeviceXRController(const InputDeviceId& inputDeviceId,
                                                     ImplementationFactory implFactoryFn)
        : InputDevice(inputDeviceId)
    {
        // Create all digital button input channels
        for (const InputChannelId& channelId : Button::All)
        {
            auto channel = aznew InputChannelDigital(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_buttonChannelsById[channelId] = channel;
        }

        // Create all analog trigger input channels
        for (const InputChannelId& channelId : Trigger::All)
        {
            auto channel = aznew InputChannelAnalog(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_triggerChannelsById[channelId] = channel;
        }

        // Create all 1D thumb-stick input channels
        for (const InputChannelId& channelId : ThumbStickAxis1D::All)
        {
            auto channel = aznew InputChannelAxis1D(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_thumbStick1DChannelsById[channelId] = channel;
        }

        // Create all 2D thumb-stick input channels
        for (const InputChannelId& channelId : ThumbStickAxis2D::All)
        {
            auto channel = aznew InputChannelAxis2D(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_thumbStick2DChannelsById[channelId] = channel;
        }

        // Create all analog thumb-stick direction input channels
        for (const InputChannelId& channelId : ThumbStickDirection::All)
        {
            auto channel = aznew InputChannelAnalog(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_thumbStickDirectionChannelsById[channelId] = channel;
        }

        // Create all 3D controller position input channels
        for (const InputChannelId& channelId : ControllerPosePosition::All)
        {
            auto channel = aznew InputChannelAxis3D(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_controllerPositionChannelsById[channelId] = channel;
        }

        // Create all Quat controller orientation input channels
        for (const InputChannelId& channelId : ControllerPoseOrientation::All)
        {
            auto channel = aznew InputChannelQuaternion(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_controllerOrientationChannelsById[channelId] = channel;
        }

        // Create the custom implementation
        SetImplementation(AZStd::move(implFactoryFn));

        // Connect to haptic feedback request bus
        InputHapticFeedbackRequestBus::Handler::BusConnect(GetInputDeviceId());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceXRController::~InputDeviceXRController()
    {
        // Disconnect from haptic feedback request bus
        InputHapticFeedbackRequestBus::Handler::BusDisconnect(GetInputDeviceId());

        // Destroy the custom implementation
        m_impl.reset();

        // Destroy all input channels
        for (const auto& channelById : m_allChannelsById)
        {
            delete channelById.second;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    const InputDevice::InputChannelByIdMap& InputDeviceXRController::GetInputChannelsById() const
    {
        return m_allChannelsById;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool InputDeviceXRController::IsSupported() const
    {
        return m_impl != nullptr;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool InputDeviceXRController::IsConnected() const
    {
        return m_impl ? m_impl->IsConnected() : false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::TickInputDevice()
    {
        if (m_impl)
        {
            m_impl->TickInputDevice();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::SetVibration(float leftMotorSpeedNormalized, float rightMotorSpeedNormalized)
    {
        if (m_impl)
        {
            m_impl->SetVibration(leftMotorSpeedNormalized, rightMotorSpeedNormalized);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::SetImplementation(AZStd::unique_ptr<Implementation> impl)
    {
        m_impl = AZStd::move(impl);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::SetImplementation(const ImplementationFactory& implFactoryFn)
    {
        if (implFactoryFn)
        {
            m_impl.reset(implFactoryFn(*this));
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceXRController::Implementation* InputDeviceXRController::GetImplementation() const
    {
        return m_impl.get();
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //! InputDeviceXRController::Implementation
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceXRController::Implementation::Implementation(InputDeviceXRController& inputDevice)
        : m_inputDevice(inputDevice)
    {
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::Implementation::BroadcastInputDeviceConnectedEvent() const
    {
        m_inputDevice.BroadcastInputDeviceConnectedEvent();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::Implementation::BroadcastInputDeviceDisconnectedEvent() const
    {
        m_inputDevice.BroadcastInputDeviceDisconnectedEvent();
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////
    //! InputDeviceXRController::Implementation::RawXRControllerState
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceXRController::Implementation::RawXRControllerState::RawXRControllerState(ButtonIdToBitMaskMap digitalButtonMap)
        : m_buttonIdsToBitMasks(AZStd::move(digitalButtonMap))
        , m_triggerMaxValue(1.f)
        , m_gripMaxValue(1.f)
        , m_thumbStickMaxValue(1.f)
    {
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::Implementation::RawXRControllerState::Reset()
    {
        m_digitalButtonStates = 0;
        m_leftTriggerState = 0.f;
        m_rightTriggerState = 0.f;
        m_leftGripState = 0.f;
        m_rightGripState = 0.f;
        m_leftThumbStickXState = 0.f;
        m_leftThumbStickYState = 0.f;
        m_rightThumbStickXState = 0.f;
        m_rightThumbStickYState = 0.f;
        m_leftPositionState = AZ::Vector3::CreateZero();
        m_rightPositionState = AZ::Vector3::CreateZero();
        m_leftOrientationState = AZ::Quaternion::CreateIdentity();
        m_rightOrientationState = AZ::Quaternion::CreateIdentity();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    float InputDeviceXRController::Implementation::RawXRControllerState::GetLeftTriggerAdjustedForDeadZoneAndNormalized() const
    {
        return AdjustForDeadZoneAndNormalizeAnalogInput(m_leftTriggerState, m_triggerDeadZoneValue, m_triggerMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    float InputDeviceXRController::Implementation::RawXRControllerState::GetRightTriggerAdjustedForDeadZoneAndNormalized() const
    {
        return AdjustForDeadZoneAndNormalizeAnalogInput(m_rightTriggerState, m_triggerDeadZoneValue, m_triggerMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    float InputDeviceXRController::Implementation::RawXRControllerState::GetLeftGripAdjustedForDeadZoneAndNormalized() const
    {
        return AdjustForDeadZoneAndNormalizeAnalogInput(m_leftGripState, m_gripDeadZoneValue, m_gripMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    float InputDeviceXRController::Implementation::RawXRControllerState::GetRightGripAdjustedForDeadZoneAndNormalized() const
    {
        return AdjustForDeadZoneAndNormalizeAnalogInput(m_rightGripState, m_gripDeadZoneValue, m_gripMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    AZ::Vector2 InputDeviceXRController::Implementation::RawXRControllerState::GetLeftThumbStickAdjustedForDeadZoneAndNormalized() const
    {
        return AdjustForDeadZoneAndNormalizeThumbStickInput(m_leftThumbStickXState, m_leftThumbStickYState,
                                                            m_leftThumbStickDeadZoneValue, m_thumbStickMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    AZ::Vector2 InputDeviceXRController::Implementation::RawXRControllerState::GetRightThumbStickAdjustedForDeadZoneAndNormalized() const
    {
        return AdjustForDeadZoneAndNormalizeThumbStickInput(m_rightThumbStickXState, m_rightThumbStickYState,
                                                            m_rightThumbStickDeadZoneValue, m_thumbStickMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    AZ::Vector2 InputDeviceXRController::Implementation::RawXRControllerState::GetLeftThumbStickNormalizedValues() const
    {
        return AZ::Vector2(m_leftThumbStickXState / m_thumbStickMaxValue,
                           m_leftThumbStickYState / m_thumbStickMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    AZ::Vector2 InputDeviceXRController::Implementation::RawXRControllerState::GetRightThumbStickNormalizedValues() const
    {
        return AZ::Vector2(m_rightThumbStickXState / m_thumbStickMaxValue,
                           m_rightThumbStickYState / m_thumbStickMaxValue);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::Implementation::ProcessRawControllerState([[maybe_unused]] const RawXRControllerState& rawControllerState)
    {
        // Update digital button channels...
        for (const auto& [channelId, bitMask] : rawControllerState.m_buttonIdsToBitMasks)
        {
            const bool buttonState = (rawControllerState.m_digitalButtonStates & bitMask) != 0;
            m_inputDevice.m_buttonChannelsById[channelId]->ProcessRawInputEvent(buttonState);
        }

        using xrc = InputDeviceXRController;

        // Update the analog triggers...
        const float triggerL = rawControllerState.GetLeftTriggerAdjustedForDeadZoneAndNormalized();
        const float triggerR = rawControllerState.GetRightTriggerAdjustedForDeadZoneAndNormalized();
        const float gripL = rawControllerState.GetLeftGripAdjustedForDeadZoneAndNormalized();
        const float gripR = rawControllerState.GetRightGripAdjustedForDeadZoneAndNormalized();
        m_inputDevice.m_triggerChannelsById[xrc::Trigger::LTrigger]->ProcessRawInputEvent(triggerL);
        m_inputDevice.m_triggerChannelsById[xrc::Trigger::RTrigger]->ProcessRawInputEvent(triggerR);
        m_inputDevice.m_triggerChannelsById[xrc::Trigger::LGrip]->ProcessRawInputEvent(gripL);
        m_inputDevice.m_triggerChannelsById[xrc::Trigger::RGrip]->ProcessRawInputEvent(gripR);

        // Update thumb-stick channels...
        const AZ::Vector2 leftThumbStick = rawControllerState.GetLeftThumbStickAdjustedForDeadZoneAndNormalized();
        const AZ::Vector2 leftThumbStickPreDeadZone = rawControllerState.GetLeftThumbStickNormalizedValues();
        const float leftStickUp = AZ::GetClamp(leftThumbStick.GetY(), s_thumbStickCenterValue, s_thumbStickMaxValue);
        const float leftStickDown = fabsf(AZ::GetClamp(leftThumbStick.GetY(), s_thumbStickMinValue, s_thumbStickCenterValue));
        const float leftStickLeft = fabsf(AZ::GetClamp(leftThumbStick.GetX(), s_thumbStickMinValue, s_thumbStickCenterValue));
        const float leftStickRight = AZ::GetClamp(leftThumbStick.GetX(), s_thumbStickCenterValue, s_thumbStickMaxValue);
        const AZ::Vector2 rightThumbStick = rawControllerState.GetRightThumbStickAdjustedForDeadZoneAndNormalized();
        const AZ::Vector2 rightThumbStickPreDeadZone = rawControllerState.GetRightThumbStickNormalizedValues();
        const float rightStickUp = AZ::GetClamp(rightThumbStick.GetY(), s_thumbStickCenterValue, s_thumbStickMaxValue);
        const float rightStickDown = fabsf(AZ::GetClamp(rightThumbStick.GetY(), s_thumbStickMinValue, s_thumbStickCenterValue));
        const float rightStickLeft = fabsf(AZ::GetClamp(rightThumbStick.GetX(), s_thumbStickMinValue, s_thumbStickCenterValue));
        const float rightStickRight = AZ::GetClamp(rightThumbStick.GetX(), s_thumbStickCenterValue, s_thumbStickMaxValue);

        m_inputDevice.m_thumbStick2DChannelsById[xrc::ThumbStickAxis2D::L]->ProcessRawInputEvent(leftThumbStick, &leftThumbStickPreDeadZone);
        m_inputDevice.m_thumbStick1DChannelsById[xrc::ThumbStickAxis1D::LX]->ProcessRawInputEvent(leftThumbStick.GetX());
        m_inputDevice.m_thumbStick1DChannelsById[xrc::ThumbStickAxis1D::LY]->ProcessRawInputEvent(leftThumbStick.GetY());
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::LU]->ProcessRawInputEvent(leftStickUp);
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::LD]->ProcessRawInputEvent(leftStickDown);
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::LL]->ProcessRawInputEvent(leftStickLeft);
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::LR]->ProcessRawInputEvent(leftStickRight);
        m_inputDevice.m_thumbStick2DChannelsById[xrc::ThumbStickAxis2D::R]->ProcessRawInputEvent(rightThumbStick, &rightThumbStickPreDeadZone);
        m_inputDevice.m_thumbStick1DChannelsById[xrc::ThumbStickAxis1D::RX]->ProcessRawInputEvent(rightThumbStick.GetX());
        m_inputDevice.m_thumbStick1DChannelsById[xrc::ThumbStickAxis1D::RY]->ProcessRawInputEvent(rightThumbStick.GetY());
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::RU]->ProcessRawInputEvent(rightStickUp);
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::RD]->ProcessRawInputEvent(rightStickDown);
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::RL]->ProcessRawInputEvent(rightStickLeft);
        m_inputDevice.m_thumbStickDirectionChannelsById[xrc::ThumbStickDirection::RR]->ProcessRawInputEvent(rightStickRight);

        // Position update...
        m_inputDevice.m_controllerPositionChannelsById[xrc::ControllerPosePosition::LPos]
            ->ProcessRawInputEvent(rawControllerState.m_leftPositionState);
        m_inputDevice.m_controllerPositionChannelsById[xrc::ControllerPosePosition::RPos]
            ->ProcessRawInputEvent(rawControllerState.m_rightPositionState);

        // TBD: Process Velocity and Acceleration...

        // Orientation update...
        m_inputDevice.m_controllerOrientationChannelsById[xrc::ControllerPoseOrientation::LOrient]
            ->ProcessRawInputEvent(rawControllerState.m_leftOrientationState);
        m_inputDevice.m_controllerOrientationChannelsById[xrc::ControllerPoseOrientation::ROrient]
            ->ProcessRawInputEvent(rawControllerState.m_rightOrientationState);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::Implementation::ResetInputChannelStates()
    {
        m_inputDevice.ResetInputChannelStates();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    AZ::u32 InputDeviceXRController::Implementation::GetInputDeviceIndex() const
    {
        return m_inputDevice.GetInputDeviceId().GetIndex();
    }

} // namespace AzFramework
