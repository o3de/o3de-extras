/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>
#include <AzFramework/Components/CameraBus.h>

#include <OpenXRVk/OpenXRVkActionsInterface.h>


namespace OpenXRVk
{
    //! XRCameraMovementComponent integrates XR Controller inputs to control a camera.
    //! This is an example that hooks up a limited set of inputs, mostly thumbsticks, to
    //! drive the camera position to new places.
    class XRCameraMovementComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public Camera::CameraNotificationBus::Handler
    {
    public:
        AZ_COMPONENT(OpenXRVk::XRCameraMovementComponent, "{7FEC0A04-D994-445C-B8DE-190D03BC3820}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus::Handler
        void OnTick(float deltaTime, AZ::ScriptTimePoint timePoint) override;

        // AzFramework::InputChannelEventListener
        // bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

        // Camera::CameraNotificationBus::Handler overrides
        void OnActiveViewChanged(const AZ::EntityId&) override;

    private:
        // void OnXRControllerEvent(const AzFramework::InputChannel& inputChannel);
        void ProcessOpenXRActions();

        // Transient data...
        AZ::Vector3 m_movement = AZ::Vector3::CreateZero();
        float m_heightZ = 0.f;

        // Serialized data...
        float m_moveSpeed = 20.f;
        float m_movementSensitivity = 0.025f;

        // We will process XR Actions only if the entity that owns this component is the active camera. 
        bool m_isActive = false;

        IOpenXRActions::ActionHandle m_moveFrontwaysHandle;
        IOpenXRActions::ActionHandle m_moveSidewaysHandle;
        IOpenXRActions::ActionHandle m_moveUpHandle;
        IOpenXRActions::ActionHandle m_moveDownHandle;
    };

} // namespace OpenXRVk
