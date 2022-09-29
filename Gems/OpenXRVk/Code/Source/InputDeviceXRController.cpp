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

////////////////////////////////////////////////////////////////////////////////////////////////////
namespace AzFramework
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // static
    bool InputDeviceXRController::IsXRControllerDevice(const InputDeviceId& inputDeviceId)
    {
        // Only check the name (crc) and ignore the index for this check.
        return (inputDeviceId.GetNameCrc32() == IdForIndex0.GetNameCrc32());
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::Reflect(AZ::ReflectContext* context)
    {
        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<InputDeviceXRController>()
                ->Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::RuntimeOwn)
                ->Constant("name", BehaviorConstant(IdForIndex0.GetName()))
                // Standard digital buttons...
                ->Constant(Button::A.GetName(), BehaviorConstant(Button::A.GetName()))
                ->Constant(Button::B.GetName(), BehaviorConstant(Button::B.GetName()))
                ->Constant(Button::X.GetName(), BehaviorConstant(Button::X.GetName()))
                ->Constant(Button::Y.GetName(), BehaviorConstant(Button::Y.GetName()))
                ->Constant(Button::Home.GetName(), BehaviorConstant(Button::Home.GetName()))
                ->Constant(Button::Menu.GetName(), BehaviorConstant(Button::Menu.GetName()))
                ->Constant(Button::L3.GetName(), BehaviorConstant(Button::L3.GetName()))
                ->Constant(Button::R3.GetName(), BehaviorConstant(Button::R3.GetName()))
                // Touch capacitive...
                ->Constant(Button::TA.GetName(), BehaviorConstant(Button::TA.GetName()))
                ->Constant(Button::TB.GetName(), BehaviorConstant(Button::TB.GetName()))
                ->Constant(Button::TX.GetName(), BehaviorConstant(Button::TX.GetName()))
                ->Constant(Button::TY.GetName(), BehaviorConstant(Button::TY.GetName()))
                ->Constant(Button::TLStick.GetName(), BehaviorConstant(Button::TLStick.GetName()))
                ->Constant(Button::TRStick.GetName(), BehaviorConstant(Button::TRStick.GetName()))
                ->Constant(Button::TLRest.GetName(), BehaviorConstant(Button::TLRest.GetName()))
                ->Constant(Button::TRRest.GetName(), BehaviorConstant(Button::TRRest.GetName()))
                ->Constant(Button::TLTrig.GetName(), BehaviorConstant(Button::TLTrig.GetName()))
                ->Constant(Button::TRTrig.GetName(), BehaviorConstant(Button::TRTrig.GetName()))
                // Analog triggers...
                ->Constant(Trigger::LTrigger.GetName(), BehaviorConstant(Trigger::LTrigger.GetName()))
                ->Constant(Trigger::RTrigger.GetName(), BehaviorConstant(Trigger::RTrigger.GetName()))
                ->Constant(Trigger::LGrip.GetName(), BehaviorConstant(Trigger::LGrip.GetName()))
                ->Constant(Trigger::RGrip.GetName(), BehaviorConstant(Trigger::RGrip.GetName()))
                // Thumbsticks (1D)...
                ->Constant(ThumbStickAxis1D::LX.GetName(), BehaviorConstant(ThumbStickAxis1D::LX.GetName()))
                ->Constant(ThumbStickAxis1D::LY.GetName(), BehaviorConstant(ThumbStickAxis1D::LY.GetName()))
                ->Constant(ThumbStickAxis1D::RX.GetName(), BehaviorConstant(ThumbStickAxis1D::RX.GetName()))
                ->Constant(ThumbStickAxis1D::RY.GetName(), BehaviorConstant(ThumbStickAxis1D::RY.GetName()))
                // Thumbsticks (2D)...
                ->Constant(ThumbStickAxis2D::L.GetName(), BehaviorConstant(ThumbStickAxis2D::L.GetName()))
                ->Constant(ThumbStickAxis2D::R.GetName(), BehaviorConstant(ThumbStickAxis2D::R.GetName()))
                // Thumbstick directions...
                ->Constant(ThumbStickDirection::LU.GetName(), BehaviorConstant(ThumbStickDirection::LU.GetName()))
                ->Constant(ThumbStickDirection::LD.GetName(), BehaviorConstant(ThumbStickDirection::LD.GetName()))
                ->Constant(ThumbStickDirection::LL.GetName(), BehaviorConstant(ThumbStickDirection::LL.GetName()))
                ->Constant(ThumbStickDirection::LR.GetName(), BehaviorConstant(ThumbStickDirection::LR.GetName()))
                ->Constant(ThumbStickDirection::RU.GetName(), BehaviorConstant(ThumbStickDirection::RU.GetName()))
                ->Constant(ThumbStickDirection::RD.GetName(), BehaviorConstant(ThumbStickDirection::RD.GetName()))
                ->Constant(ThumbStickDirection::RL.GetName(), BehaviorConstant(ThumbStickDirection::RL.GetName()))
                ->Constant(ThumbStickDirection::RR.GetName(), BehaviorConstant(ThumbStickDirection::RR.GetName()))
                // Position (3D)...
                ->Constant(ControllerPosePosition::LPos.GetName(), BehaviorConstant(ControllerPosePosition::LPos.GetName()))
                ->Constant(ControllerPosePosition::RPos.GetName(), BehaviorConstant(ControllerPosePosition::RPos.GetName()))
                //->Constant(ControllerPosePosition::LVel.GetName(), BehaviorConstant(ControllerPosePosition::LVel.GetName()))
                //->Constant(ControllerPosePosition::RVel.GetName(), BehaviorConstant(ControllerPosePosition::RVel.GetName()))
                //->Constant(ControllerPosePosition::LAcc.GetName(), BehaviorConstant(ControllerPosePosition::LAcc.GetName()))
                //->Constant(ControllerPosePosition::RAcc.GetName(), BehaviorConstant(ControllerPosePosition::RAcc.GetName()))
                // Orientation (quaternion)...
                ->Constant(ControllerPoseOrientation::LOrient.GetName(), BehaviorConstant(ControllerPoseOrientation::LOrient.GetName()))
                ->Constant(ControllerPoseOrientation::ROrient.GetName(), BehaviorConstant(ControllerPoseOrientation::ROrient.GetName()))
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
            const auto channel = aznew InputChannelDigital(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_buttonChannelsById[channelId] = channel;
        }

        // Create all analog trigger input channels
        for (const InputChannelId& channelId : Trigger::All)
        {
            const auto channel = aznew InputChannelAnalog(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_triggerChannelsById[channelId] = channel;
        }

        // Create all 1D thumb-stick input channels
        for (const InputChannelId& channelId : ThumbStickAxis1D::All)
        {
            const auto channel = aznew InputChannelAxis1D(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_thumbStick1DChannelsById[channelId] = channel;
        }

        // Create all 2D thumb-stick input channels
        for (const InputChannelId& channelId : ThumbStickAxis2D::All)
        {
            const auto channel = aznew InputChannelAxis2D(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_thumbStick2DChannelsById[channelId] = channel;
        }

        // Create all analog thumb-stick direction input channels
        for (const InputChannelId& channelId : ThumbStickDirection::All)
        {
            const auto channel = aznew InputChannelAnalog(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_thumbStickDirectionChannelsById[channelId] = channel;
        }

        // Create all 3D controller position input channels
        for (const InputChannelId& channelId : ControllerPosePosition::All)
        {
            const auto channel = aznew InputChannelAxis3D(channelId, *this);
            m_allChannelsById[channelId] = channel;
            m_controllerPositionChannelsById[channelId] = channel;
        }

        // Create all Quat controller orientation input channels
        for (const InputChannelId& channelId : ControllerPoseOrientation::All)
        {
            const auto channel = aznew InputChannelQuaternion(channelId, *this);
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
        m_pimpl.reset();

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
        return m_pimpl != nullptr;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    bool InputDeviceXRController::IsConnected() const
    {
        return m_pimpl ? m_pimpl->IsConnected() : false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::TickInputDevice()
    {
        if (m_pimpl)
        {
            m_pimpl->TickInputDevice();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::SetVibration(float leftMotorSpeedNormalized, float rightMotorSpeedNormalized)
    {
        if (m_pimpl)
        {
            m_pimpl->SetVibration(leftMotorSpeedNormalized, rightMotorSpeedNormalized);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::SetImplementation(AZStd::unique_ptr<Implementation> impl)
    {
        m_pimpl = AZStd::move(impl);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    void InputDeviceXRController::SetImplementation(const ImplementationFactory& implFactoryFn)
    {
        if (implFactoryFn)
        {
            m_pimpl.reset(implFactoryFn(*this));
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    InputDeviceXRController::Implementation* InputDeviceXRController::GetImplementation() const
    {
        return m_pimpl.get();
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
        for (const auto& [channelIdPtr, bitMask] : rawControllerState.m_buttonIdsToBitMasks)
        {
            const bool buttonState = (rawControllerState.m_digitalButtonStates & bitMask) != 0;
            m_inputDevice.m_buttonChannelsById[*channelIdPtr]->ProcessRawInputEvent(buttonState);
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
        const float leftStickUp = AZ::GetClamp(leftThumbStick.GetY(), 0.f, 1.f);
        const float leftStickDown = fabsf(AZ::GetClamp(leftThumbStick.GetY(), -1.f, 0.f));
        const float leftStickLeft = fabsf(AZ::GetClamp(leftThumbStick.GetX(), -1.f, 0.f));
        const float leftStickRight = AZ::GetClamp(leftThumbStick.GetX(), 0.f, 1.f);
        const AZ::Vector2 rightThumbStick = rawControllerState.GetRightThumbStickAdjustedForDeadZoneAndNormalized();
        const AZ::Vector2 rightThumbStickPreDeadZone = rawControllerState.GetRightThumbStickNormalizedValues();
        const float rightStickUp = AZ::GetClamp(rightThumbStick.GetY(), 0.f, 1.f);
        const float rightStickDown = fabsf(AZ::GetClamp(rightThumbStick.GetY(), -1.f, 0.f));
        const float rightStickLeft = fabsf(AZ::GetClamp(rightThumbStick.GetX(), -1.f, 0.f));
        const float rightStickRight = AZ::GetClamp(rightThumbStick.GetX(), 0.f, 1.f);

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
