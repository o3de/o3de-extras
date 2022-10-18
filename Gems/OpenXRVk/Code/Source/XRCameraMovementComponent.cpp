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

namespace OpenXRVk
{
    void XRCameraMovementComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<XRCameraMovementComponent, AZ::Component>()
                ->Version(1)
                ->Field("Move Speed", &XRCameraMovementComponent::m_moveSpeed)
                ->Field("Rotation Speed", &XRCameraMovementComponent::m_rotationSpeed)
                ->Field("Movement Sensitivity", &XRCameraMovementComponent::m_movementSensitivity)
                ->Field("Rotation Sensitivity", &XRCameraMovementComponent::m_rotationSensitivity)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<XRCameraMovementComponent>("XRCameraMovementComponent", "[Description of functionality provided by this component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ComponentCategory")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Icons/Components/Component_Placeholder.svg")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &XRCameraMovementComponent::m_moveSpeed, "Move Speed", "Speed of camera movement")
                    ->Attribute(AZ::Edit::Attributes::Min, 1.f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &XRCameraMovementComponent::m_rotationSpeed, "Rotation Speed", "Speed of camera rotation")
                    ->Attribute(AZ::Edit::Attributes::Min, 1.f)
                    ->Attribute(AZ::Edit::Attributes::Max, 100.f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &XRCameraMovementComponent::m_movementSensitivity, "Move Sensitivity", "Fine movement sensitivity factor")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.f)
                    ->Attribute(AZ::Edit::Attributes::Max, 1.f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &XRCameraMovementComponent::m_rotationSensitivity, "Rotation Sensitivity", "Fine rotation sensitivity factor")
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
        XRCameraMovementRequestBus::Handler::BusConnect(GetEntityId());
    }

    void XRCameraMovementComponent::Deactivate()
    {
        XRCameraMovementRequestBus::Handler::BusDisconnect(GetEntityId());
        AZ::TickBus::Handler::BusDisconnect();
        AzFramework::InputChannelEventListener::Disconnect();
    }

    void XRCameraMovementComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint timePoint)
    {
        AZ::Transform worldTransform{};
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        // Update movement...
        const float moveSpeed = m_moveSpeed * deltaTime;
        const AZ::Vector3 movementVec = (worldTransform.GetBasisY() * m_movement.GetY())
            + (worldTransform.GetBasisX() * m_movement.GetX())
            + (worldTransform.GetBasisZ() * m_movement.GetZ());
        const AZ::Vector3 newPosition{ (worldTransform.GetTranslation() + (movementVec * moveSpeed)) };
        worldTransform.SetTranslation(newPosition);

        // Update rotation...
        const float rotateSpeed = m_rotationSpeed * deltaTime;
        const AZ::Quaternion orientation{ worldTransform.GetRotation() };
        // First off, only perform a rotation about Z axis (based on right-thumbstick X-axis)
        const AZ::Vector3 rotationDegrees{ 0.f, 0.f, (rotateSpeed * m_rotation.GetX()) };
        const AZ::Quaternion newOrientation{ orientation * AZ::Quaternion::CreateFromEulerAnglesDegrees(rotationDegrees) };
        worldTransform.SetRotation(newOrientation);

        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, worldTransform);
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

        // Left thumb-stick X/Y move the camera
        if (channelId == AzFramework::InputDeviceXRController::ThumbStickAxis1D::LX)
        {
            m_movement.SetX(inputChannel.GetValue() * m_movementSensitivity);
        }
        if (channelId == AzFramework::InputDeviceXRController::ThumbStickAxis1D::LY)
        {
            m_movement.SetY(inputChannel.GetValue() * m_movementSensitivity);
        }

        // Right thumb-stick X/Y rotates the camera
        if (channelId == AzFramework::InputDeviceXRController::ThumbStickAxis1D::RX)
        {
            m_rotation.SetX(inputChannel.GetValue() * m_rotationSensitivity);
        }
        if (channelId == AzFramework::InputDeviceXRController::ThumbStickAxis1D::RY)
        {
            m_rotation.SetY(inputChannel.GetValue() * m_rotationSensitivity);
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
