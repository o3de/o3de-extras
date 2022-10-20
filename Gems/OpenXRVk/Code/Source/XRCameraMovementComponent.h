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


namespace OpenXRVk
{
    //! XRCameraMovementComponent integrates XR Controller inputs to control a camera.
    //! This is an example that hooks up a limited set of inputs, mostly thumbsticks, to
    //! drive the camera position to new places.
    class XRCameraMovementComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AzFramework::InputChannelEventListener
    {
    public:
        AZ_COMPONENT(OpenXRVk::XRCameraMovementComponent, "{7FEC0A04-D994-445C-B8DE-190D03BC3820}");

        //! Reflects component data into the reflection contexts, including the serialization, edit, and behavior contexts.
        static void Reflect(AZ::ReflectContext* context);

        //! Specifies the services that this component provides.
        //! Other components can declare a dependency on these services. The system uses this
        //! information to determine the order of component activation.
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        //! Specifies the services that this component cannot operate with.
        //! For example, if two components provide a similar service and the system cannot use the
        //! services simultaneously, each of those components would specify the other component as
        //! an incompatible service.
        //! If an entity cannot have multiple instances of this component, include this component's
        //! provided service in the list of incompatible services.
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        //! Specifies the services that this component requires.
        //! The system activates the required services before it activates this component.
        //! It also deactivates the required services after it deactivates this component.
        //! If a required service is missing before this component is activated, the system
        //! returns an error and does not activate this component.
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Specifies the services that this component depends on, but does not require.
        //! The system activates the dependent services before it activates this component.
        //! It also deactivates the dependent services after it deactivates this component.
        //! If a dependent service is missing before this component is activated, the system
        //! does not return an error and still activates this component.
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

    protected:
        //! Puts this component into an active state.
        //! The system calls this function once during activation of each entity that owns the
        //! component. You must override this function. The system calls a component's Activate()
        //! function only if all services and components that the component depends on are present
        //! and active.
        void Activate() override;

        //! Deactivates this component.
        //! The system calls this function when the owning entity is being deactivated. You must
        //! override this function. As a best practice, ensure that this function returns this component
        //! to a minimal footprint. The order of deactivation is the reverse of activation, so your
        //! component is deactivated before the components it depends on.
        //! The system always calls a component's Deactivate() function before destroying the component.
        //! However, deactivation is not always followed by the destruction of the component. An entity and
        //! its components can be deactivated and reactivated without being destroyed. Ensure that your
        //! Deactivate() implementation can handle this scenario.
        void Deactivate() override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint timePoint) override;

        // AzFramework::InputChannelEventListener
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

    private:
        void OnXRControllerEvent(const AzFramework::InputChannel& inputChannel);
        static AZ::Transform GetCameraTransformFromCurrentView();

        // Transient data...
        AZ::Vector3 m_movement = AZ::Vector3::CreateZero();
        float m_heightZ = 0.f;

        // Serialized data...
        float m_moveSpeed = 20.f;
        float m_movementSensitivity = 0.025f;
    };

} // namespace OpenXRVk
