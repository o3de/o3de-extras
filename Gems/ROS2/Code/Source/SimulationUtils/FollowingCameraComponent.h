/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "FollowingCameraConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Input/Buses/Requests/InputSystemCursorRequestBus.h>
#include <AzFramework/Input/Devices/Keyboard/InputDeviceKeyboard.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>

namespace std
{
    template<>
    struct hash<AzFramework::InputChannelId>
    {
        std::size_t operator()(const AzFramework::InputChannelId& channelId) const noexcept
        {
            return std::hash<std::string>()(channelId.GetName());
        }
    };
} // namespace std

struct InputChannelIdHash
{
    std::size_t operator()(const AzFramework::InputChannelId& channelId) const noexcept
    {
        return std::hash<std::string>()(channelId.GetName());
    }
};

namespace ROS2
{
    using Key = AzFramework::InputDeviceKeyboard::Key;

    //! The component used for cameras that follow moving objects
    //! It allows to switch between different cameras attached to entities, and to control the active camera using keyboard.
    class FollowingCameraComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AzFramework::InputChannelEventListener
    {
    public:
        FollowingCameraComponent() = default;
        FollowingCameraComponent(const FollowingCameraConfiguration& configuration);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        static void Reflect(AZ::ReflectContext* reflection);

        AZ_COMPONENT(FollowingCameraComponent, "{6a21768a-f327-11ed-a05b-0242ac120003}", AZ::Component);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

    private:
        // AZ::TickBus overrides ..
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AzFramework::InputChannelEventListener overrides ...
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;
        void keyboardEvent(const AzFramework::InputChannel& inputChannel);
        void mouseEvent(const AzFramework::InputChannel& inputChannel);

        //! Rotate the camera in the screen space.
        //! @param mouseDelta The mouse move delta in the screen space.
        void RotateCameraOnMouse(const AZ::Vector2& mouseDelta);

        //! Get the current mouse position (2D screen space).
        AZ::Vector2 GetCurrentMousePosition();

        //! Move the camera on the keys WSAD.
        void MoveCameraOnKeys();

        //! Rotate the camera around the axis up (yaw rotation) QE.
        void RotateCameraOnKeys();

        //! Get the pose of the entity.
        //! @param entityId The entity to get the pose.
        //! @return The pose of the entity (local transform).
        AZ::Matrix4x4 GetEntityLocalPose(AZ::EntityId entityId);

        //! Remove the tilt from the transform.
        //! The tilt is removed by projecting the transform basis of rotation matrix to the ground plane.
        //! @param transform The transform to remove the tilt.
        //! @return The transform without tilt.
        AZ::Transform RemoveTiltFromTransform(AZ::Transform transform);

        //! Compute weighted average of the vectors in the buffer.
        //! @param buffer The buffer to compute the average.
        //! @return The average vector.
        AZ::Vector3 WeightedAverageVector(const AZStd::deque<AZStd::pair<AZ::Vector3, float>>& buffer) const;

        //! Compute weighted average of translation in the buffer.
        //! @return The average translation.
        AZ::Vector3 SmoothTranslation() const;

        //! @return The average rotation.
        AZ::Quaternion SmoothRotation() const;

        //! Compute weighted average of rotation in the buffer.
        AZ::Quaternion CalculateSmoothedRotation() const;

        //! Cache the transform in smoothing buffer.
        //! @param transform The transform to cache.
        //! @param deltaTime The time between the last frame and the current frame.
        void CacheTransform(const AZ::Transform& transform, float deltaTime);

        void FreezeCamera(); //! Freeze the camera in current position.
        void ResumeCamera(); //! Resume the camera to follow the target.

        //! The smoothing buffer for translation, the first element is the translation, the second element is the weight.
        AZStd::deque<AZStd::pair<AZ::Vector3, float>> m_lastTranslationsBuffer;

        //! The smoothing buffer for rotation, the first element is the tangential vector, the second element is the weight.
        AZStd::deque<AZStd::pair<AZ::Quaternion, float>> m_lastRotationsBuffer;

        float m_opticalAxisTranslation = 0.0f; //!< The zoom change from the input.
        AZ::EntityId m_currentView; //!< Current used view point.
        AZ::Transform m_frozenTransform;
        bool m_isFollowingEnabled = true;
        bool m_isRightMouseButtonPressed = false; //!< Apply mouse rotation whether the right mouse button is pressed.
        AZ::Vector2 m_initialMousePosition; //!< The initial mouse position when the right mouse button is pressed.
        AZ::Vector2 m_lastMousePosition = AZ::Vector2(0.0f, 0.0f);
        float m_mouseDeltaX = 0.0f;
        float m_mouseDeltaY = 0.0f;

        bool m_ignoreNextMovement = false; //!< Ignore the next movement to avoid camera jump.

        AZ::Matrix4x4 m_cameraOffset = AZ::Matrix4x4::CreateIdentity(); //!< The current relative camera pose.
        std::unordered_map<AzFramework::InputChannelId, bool, InputChannelIdHash> m_keyStates; //!< For multiple key press detection.

        // smoothing camera
        AZ::Vector3 m_cameraVelocity = AZ::Vector3::CreateZero(); //! Camera's current velocity
        float m_springConstant = 50.0f; //! Spring constant (springiness)
        float m_dampingConstant = 5.0f; //! Damping constant (resistance)

        FollowingCameraConfiguration m_configuration; //!< The configuration of the following camera.

        // The keys for camera movement and rotation.

        const std::unordered_set<AzFramework::InputChannelId> moveKeys = {
            Key::AlphanumericW, Key::AlphanumericS, Key::AlphanumericA, Key::AlphanumericD
        };
        const std::unordered_set<AzFramework::InputChannelId> rotateKeys = { Key::AlphanumericQ, Key::AlphanumericE };
        const std::unordered_set<AzFramework::InputChannelId> shiftKeys = { Key::ModifierShiftL, Key::ModifierShiftR };
    };

} // namespace ROS2
