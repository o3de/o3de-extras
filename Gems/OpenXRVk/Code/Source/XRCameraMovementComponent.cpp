/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <XRCameraMovementComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/RTTI/BehaviorContext.h>

#include <AzCore/Math/MathStringConversions.h> // GALIB

#include <OpenXRVk/InputDeviceXRController.h>
#include <OpenXRVk/OpenXRActionsInterface.h>

#include <Atom/RPI.Public/ViewProviderBus.h>
#include <Atom/RPI.Public/View.h>
#include <AzFramework/Components/CameraBus.h>

#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>


namespace OpenXRVk
{
    void XRCameraMovementComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<XRCameraMovementComponent, AZ::Component>()
                ->Version(1)
                ->Field("Move Speed", &XRCameraMovementComponent::m_moveSpeed)
                ->Field("Movement Sensitivity", &XRCameraMovementComponent::m_movementSensitivity)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<XRCameraMovementComponent>("XR Camera Movement", "Provides XR controller input to control the camera")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "Gameplay")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Icons/Components/Component_Placeholder.svg")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &XRCameraMovementComponent::m_moveSpeed, "Move Speed", "Speed of camera movement")
                        ->Attribute(AZ::Edit::Attributes::Min, 1.f)
                        ->Attribute(AZ::Edit::Attributes::Max, 50.f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &XRCameraMovementComponent::m_movementSensitivity, "Move Sensitivity", "Fine movement sensitivity factor")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.f)
                        ->Attribute(AZ::Edit::Attributes::Max, 1.f)
                    ;
            }
        }

        if (auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<XRCameraMovementComponent>("XRCameraMovement Component Group")
                ->Attribute(AZ::Script::Attributes::Category, "OpenXRVk Gem Group")
                ;
        }
    }

    void XRCameraMovementComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("CameraMovementService"));
    }

    void XRCameraMovementComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("CameraMovementService"));
    }

    void XRCameraMovementComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void XRCameraMovementComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void XRCameraMovementComponent::Activate()
    {
        Camera::CameraNotificationBus::Handler::BusConnect();
        if (m_isActive)
        {
            AzFramework::InputChannelEventListener::Connect();
            AZ::TickBus::Handler::BusConnect();
        }
    }

    void XRCameraMovementComponent::Deactivate()
    {
        if (AZ::TickBus::Handler::BusIsConnected())
        {
            AZ::TickBus::Handler::BusDisconnect();
        }
        if (AzFramework::InputChannelEventListener::BusIsConnected())
        {
            AzFramework::InputChannelEventListener::Disconnect();
        }
        
        Camera::CameraNotificationBus::Handler::BusDisconnect();
    }

    void XRCameraMovementComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        ProcessOpenXRActions();

        AZ::Transform cameraTransform;
        AZ::TransformBus::EventResult(cameraTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        // Update movement...
        const float moveSpeed = m_moveSpeed * deltaTime;
        const AZ::Vector3 movementVec = 
              (cameraTransform.GetBasisX() * m_movement.GetX())
            + (cameraTransform.GetBasisY() * m_movement.GetY())
            + (cameraTransform.GetBasisZ() * m_movement.GetZ());
        const AZ::Vector3 newPosition{ (cameraTransform.GetTranslation() + (movementVec * moveSpeed)) };
        cameraTransform.SetTranslation(newPosition);

        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, cameraTransform);
    }

    bool XRCameraMovementComponent::OnInputChannelEventFiltered([[maybe_unused]] const AzFramework::InputChannel& inputChannel)
    {
        const auto& deviceId = inputChannel.GetInputDevice().GetInputDeviceId();
        if (AzFramework::InputDeviceXRController::IsXRControllerDevice(deviceId))
        {
            OnXRControllerEvent(inputChannel);
        }
        return false;
    }

    void XRCameraMovementComponent::OnXRControllerEvent([[maybe_unused]] const AzFramework::InputChannel& inputChannel)
    {
        const auto& channelId = inputChannel.GetInputChannelId();

        // This currently uses specific xr controller channels to drive the movement.  Future iterations might
        // use a higher-level concepts like InputMappings and InputContexts to generalize to additional
        // input devices.

        // Left thumb-stick X/Y move the camera
        if (channelId == AzFramework::InputDeviceXRController::ThumbStickAxis1D::LX)
        {
            m_movement.SetX(inputChannel.GetValue() * m_movementSensitivity);
        }
        if (channelId == AzFramework::InputDeviceXRController::ThumbStickAxis1D::LY)
        {
            m_movement.SetY(inputChannel.GetValue() * m_movementSensitivity);
        }

        // A/B buttons update the height in Z of the camera
        if (channelId == AzFramework::InputDeviceXRController::Button::A)
        {   // down
            m_movement.SetZ(-inputChannel.GetValue() * m_movementSensitivity);
        }
        if (channelId == AzFramework::InputDeviceXRController::Button::B)
        {   // up
            m_movement.SetZ(inputChannel.GetValue() * m_movementSensitivity);
        }
    }

    void XRCameraMovementComponent::ProcessOpenXRActions()
    {
        auto actionsIFace = OpenXRActionsInterface::Get();
        if (!actionsIFace)
        {
            return;
        }
        // Button
        {
            auto actionHandle = actionsIFace->GetActionHandle("my_action_set", "button");
            if (!actionHandle.IsValid())
            {
                return;
            }
            auto outcome = actionsIFace->GetActionStateBoolean(actionHandle);
            if (outcome.IsSuccess())
            {
                if (outcome.GetValue())
                {
                    // up
                    m_movement.SetZ(m_movementSensitivity);
                }
            }
        }
        // Pose
        {
            auto actionHandle = actionsIFace->GetActionHandle("my_action_set", "left_pose");
            if (!actionHandle.IsValid())
            {
                return;
            }
            auto outcome = actionsIFace->GetActionStatePose(actionHandle);
            if (outcome.IsSuccess())
            {
                AZ::Transform tm(outcome.TakeValue());
                AZ_Printf("Galib", "left_pose tm=\n%s\n", AZStd::to_string(tm).c_str());
            }
        }

    }

    // Camera::CameraNotificationBus::Handler overrides
    void XRCameraMovementComponent::OnActiveViewChanged(const AZ::EntityId& activeEntityId)
    {
        m_isActive = activeEntityId == GetEntityId();
        if (m_isActive)
        {
            if (!AZ::TickBus::Handler::BusIsConnected())
            {
                AZ::TickBus::Handler::BusConnect();
            }
            if (!AzFramework::InputChannelEventListener::BusIsConnected())
            {
                AzFramework::InputChannelEventListener::Connect();
            }
        }
        else
        {
            if (AZ::TickBus::Handler::BusIsConnected())
            {
                AZ::TickBus::Handler::BusDisconnect();
            }
            if (AzFramework::InputChannelEventListener::BusIsConnected())
            {
                AzFramework::InputChannelEventListener::Disconnect();
            }
        }
    }

} // namespace OpenXRVk
