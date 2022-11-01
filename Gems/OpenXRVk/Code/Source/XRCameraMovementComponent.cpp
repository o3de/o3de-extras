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

#include <OpenXRVk/InputDeviceXRController.h>

#include <Atom/RPI.Public/ViewProviderBus.h>
#include <Atom/RPI.Public/View.h>
#include <AzFramework/Components/CameraBus.h>

#include <Atom/RPI.Public/ViewportContext.h>
#include <Atom/RPI.Public/ViewportContextBus.h>


namespace OpenXRVk
{
    static AZ::Transform GetCameraTransformFromCurrentView()
    {
        if (const auto viewportContextMgr = AZ::Interface<AZ::RPI::ViewportContextRequestsInterface>::Get();
            viewportContextMgr != nullptr)
        {
            if (const AZ::RPI::ViewportContextPtr viewportContext = viewportContextMgr->GetDefaultViewportContext();
                viewportContext != nullptr)
            {
                if (const AZ::RPI::ViewPtr view = viewportContext->GetDefaultView();
                    view != nullptr)
                {
                    return view->GetCameraTransform();
                }
            }
        }
        return AZ::Transform::CreateIdentity();
    }

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
        AzFramework::InputChannelEventListener::Connect();
        AZ::TickBus::Handler::BusConnect();
    }

    void XRCameraMovementComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        AzFramework::InputChannelEventListener::Disconnect();
    }

    void XRCameraMovementComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        AZ::Transform cameraTransform = GetCameraTransformFromCurrentView();

        // Update movement...
        const float moveSpeed = m_moveSpeed * deltaTime;
        const AZ::Vector3 movementVec = (cameraTransform.GetBasisX() * m_movement.GetX())
            + (cameraTransform.GetBasisY() * m_movement.GetY())
            + (AZ::Vector3{0.f, 0.f, 1.f} * m_movement.GetZ()); // use a fixed UP for the Z direction
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

} // namespace OpenXRVk
