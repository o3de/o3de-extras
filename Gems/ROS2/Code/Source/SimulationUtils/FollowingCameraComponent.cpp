/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FollowingCameraComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Input/Devices/Mouse/InputDeviceMouse.h>
#include <MathConversion.h>

namespace ROS2
{
    using Key = AzFramework::InputDeviceKeyboard::Key;
    using Movement = AzFramework::InputDeviceMouse::Movement;

    // Default keyboard mapping for predefined views.
    const AZStd::unordered_map<AzFramework::InputChannelId, int> KeysToView{ { Key::Alphanumeric1, 0 }, { Key::Alphanumeric2, 1 },
                                                                             { Key::Alphanumeric3, 2 }, { Key::Alphanumeric4, 3 },
                                                                             { Key::Alphanumeric5, 4 }, { Key::Alphanumeric6, 5 },
                                                                             { Key::Alphanumeric7, 6 }, { Key::Alphanumeric8, 7 },
                                                                             { Key::Alphanumeric9, 8 }, { Key::Alphanumeric0, 9 } };

    FollowingCameraComponent::FollowingCameraComponent(const FollowingCameraConfiguration& configuration)
        : m_configuration(configuration)
    {
    }

    void FollowingCameraComponent::Reflect(AZ::ReflectContext* reflection)
    {
        FollowingCameraConfiguration::Reflect(reflection);
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(reflection);
        if (serializeContext)
        {
            serializeContext->Class<FollowingCameraComponent, AZ::Component>()->Version(1)->Field(
                "FollowingCameraConfiguration", &FollowingCameraComponent::m_configuration);
            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<FollowingCameraComponent>("Following Camera", "Camera following entity with predefined views")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2 Utilities")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/Camera.svg")
                    ->UIElement(AZ::Edit::UIHandlers::Label, "", "")
                    ->Attribute(
                        AZ::Edit::Attributes::ValueText,
                        "This Component allows to switch camera view between predefined views. "
                        "It also allows to zoom in/out and rotate around parent transformation. "
                        "Use 0-9 keys to switch views and W, S, A, D keys to manipulate current view.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraComponent::m_configuration,
                        "FollowingCameraConfiguration",
                        "FollowingCameraConfiguration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void FollowingCameraComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("FollowingCameraService"));
    }

    void FollowingCameraComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("FollowingCameraService"));
    }

    void FollowingCameraComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService"));
        required.push_back(AZ_CRC("CameraService"));
    }

    void FollowingCameraComponent::Activate()
    {
        InputChannelEventListener::Connect();
        AZ::TickBus::Handler::BusConnect();

        if (m_configuration.m_predefinedViews.size() == 0)
        {
            AZ_Warning("FollowingCameraComponent", false, "No predefined views");
            return;
        }
        if (m_configuration.m_defaultView < m_configuration.m_predefinedViews.size())
        {
            m_currentView = m_configuration.m_predefinedViews[m_configuration.m_defaultView];
            m_cameraOffset = GetEntityLocalPose(m_currentView);
        }
    }

    void FollowingCameraComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputChannelEventListener::Disconnect();
    }

    AZ::Transform FollowingCameraComponent::RemoveTiltFromTransform(AZ::Transform transform)
    {
        const AZ::Vector3 axisX = transform.GetBasisX();
        const AZ::Vector3 axisY = transform.GetBasisY();

        const AZ::Matrix3x3 projectionOnXY{ AZ::Matrix3x3::CreateFromColumns(
            AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateZero()) };

        const AZ::Vector3 newAxisZ = AZ::Vector3::CreateAxisZ(); // new axis Z points up

        // project axisX on the XY plane
        const AZ::Vector3 projectedAxisX = (projectionOnXY * axisX);
        const AZ::Vector3 projectedAxisY = (projectionOnXY * axisY);

        AZ::Vector3 newAxisX = AZ::Vector3::CreateZero();
        AZ::Vector3 newAxisY = AZ::Vector3::CreateZero();

        // get 3rd vector of the new basis from the cross product of the projected vectors.
        // Primarily we want to use the projectedAxisX as the newAxisX, but if it is zero-length, we use the projectedAxisY as the newAxisY.
        if (!projectedAxisX.IsZero())
        {
            newAxisX = projectedAxisX.GetNormalized();
            newAxisY = newAxisZ.Cross(newAxisX);
        }
        else
        {
            newAxisY = projectedAxisY.GetNormalized();
            newAxisX = newAxisY.Cross(newAxisZ);
        }
        // apply rotation using created basis
        transform.SetRotation(AZ::Quaternion::CreateFromBasis(newAxisX, newAxisY, newAxisZ));
        return transform;
    }

    void FollowingCameraComponent::CacheTransform(const AZ::Transform& targetTransform, float deltaTime)
    {
        // Convert targetTransform to Matrix4x4 for consistency with m_cameraOffset
        AZ::Matrix4x4 targetMatrix = AZ::Matrix4x4::CreateFromTransform(targetTransform);

        // Combine the target transform with the camera offset to get the camera's world pose
        // m_cameraOffset already contains translation and rotation changes applied by keyboard inputs
        AZ::Matrix4x4 cameraWorldPose = targetMatrix * m_cameraOffset;

        // Update the smoothing buffers
        // Extract translation and rotation from cameraWorldPose
        AZ::Vector3 translation = cameraWorldPose.GetTranslation();
        AZ::Quaternion rotation = AZ::Quaternion::CreateFromMatrix4x4(cameraWorldPose);

        m_lastTranslationsBuffer.push_back(AZStd::make_pair(translation, deltaTime));
        m_lastRotationsBuffer.push_back(AZStd::make_pair(rotation, deltaTime));

        // Ensure the buffers do not exceed the configured smoothing buffer size
        if (m_lastTranslationsBuffer.size() > m_configuration.m_smoothingBuffer)
        {
            m_lastTranslationsBuffer.pop_front();
        }
        if (m_lastRotationsBuffer.size() > m_configuration.m_smoothingBuffer)
        {
            m_lastRotationsBuffer.pop_front();
        }
    }

    void FollowingCameraComponent::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        // m_currentView refers to camera ID
        AZ_Warning("FollowingCameraComponent", m_currentView.IsValid(), "View is not valid");
        if (!m_currentView.IsValid())
        {
            return;
        }

        // Follow the target
        // obtain the current view transform
        AZ::Transform target_local_transform;
        AZ::Transform target_world_transform;

        AZ::TransformBus::Event(m_currentView, &AZ::TransformBus::Events::GetLocalAndWorld, target_local_transform, target_world_transform);

        // get parent's transform
        const AZ::Transform parent_transform = target_world_transform * target_local_transform.GetInverse();

        const AZ::Transform transformToCache = m_configuration.m_lockZAxis ? RemoveTiltFromTransform(parent_transform) : parent_transform;
        CacheTransform(transformToCache, deltaTime);

        // get the averaged translation and quaternion
        AZ::Transform filtered_parent_transform = { SmoothTranslation(), SmoothRotation(), 1.f };

        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, filtered_parent_transform);
    }

    AZ::Vector3 FollowingCameraComponent::WeightedAverageVector(const AZStd::deque<AZStd::pair<AZ::Vector3, float>>& buffer) const
    {
        AZ::Vector3 weightedSum = AZ::Vector3::CreateZero();
        float totalWeight = 0.0f;
        float currentWeight = 1.0f; // Start with a weight of 1 for the oldest entry

        for (const auto& entry : buffer)
        {
            weightedSum += entry.first * currentWeight; // Apply current weight to the vector
            totalWeight += currentWeight; // Accumulate total weight for normalization
            currentWeight *= m_configuration.m_smoothFactor; // Increase weight for the next, more recent entry
        }

        if (totalWeight > 0.0f)
        {
            return weightedSum / totalWeight; // Normalize weighted sum by total weight
        }

        return AZ::Vector3::CreateZero(); // Return zero vector if buffer is empty or totalWeight is somehow zero
    }

    AZ::Vector3 FollowingCameraComponent::SmoothTranslation() const
    {
        // Use the modified weighted average calculation for smoothing
        return WeightedAverageVector(m_lastTranslationsBuffer);
    }

    AZ::Quaternion FollowingCameraComponent::SmoothRotation() const
    {
        if (m_lastRotationsBuffer.empty())
        {
            return AZ::Quaternion::CreateIdentity();
        }

        return CalculateSmoothedRotation();
    }

    AZ::Quaternion FollowingCameraComponent::CalculateSmoothedRotation() const
    {
        // Start with the oldest rotation in the buffer
        AZ::Quaternion smoothedRotation = m_lastRotationsBuffer.front().first;
        float totalWeight = 0.0f;
        float currentWeight = 1.0f; // Initial weight

        for (size_t i = 1; i < m_lastRotationsBuffer.size(); ++i)
        {
            totalWeight += currentWeight;
            float t = currentWeight / totalWeight; // Calculate interpolation factor

            AZ_Assert(t >= 0.0f && t <= 1.0f, "Interpolation factor must be in the range [0, 1]");

            // Progressively slerp towards each newer rotation, with increasing influence
            // Interpolates between series of rotations to get a smooth rotation with new rotations having more influence (less lag)
            smoothedRotation = smoothedRotation.Slerp(m_lastRotationsBuffer[i].first, t);

            currentWeight *= m_configuration.m_smoothFactor; // Increase the weight for the next rotation
        }

        return smoothedRotation;
    }

    bool FollowingCameraComponent::OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputDeviceId& deviceId = inputChannel.GetInputDevice().GetInputDeviceId();

        // Handle mouse events
        if (AzFramework::InputDeviceMouse::IsMouseDevice(deviceId))
        {
            MouseEvent(inputChannel);
        }

        if (AzFramework::InputDeviceKeyboard::IsKeyboardDevice(deviceId))
        {
            KeyboardEvent(inputChannel);
        }

        return false;
    }

    void FollowingCameraComponent::MouseEvent(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputChannelId& channelId = inputChannel.GetInputChannelId();

        if (channelId == AzFramework::InputDeviceMouse::Button::Right)
        {
            if (inputChannel.IsStateBegan())
            {
                m_isRightMouseButtonPressed = true;

                // Capture the initial mouse position and cursor state
                m_initialMousePosition = GetCurrentMousePosition();
                m_ignoreNextMovement = true; // Flag to ignore the next mouse movement preventing jump on re-centering
            }
            else if (inputChannel.IsStateEnded())
            {
                m_isRightMouseButtonPressed = false;

                // Restore the cursor's original position
                AzFramework::InputSystemCursorRequestBus::Event(
                    AzFramework::InputDeviceMouse::Id,
                    &AzFramework::InputSystemCursorRequests::SetSystemCursorPositionNormalized,
                    m_initialMousePosition);

                // Update m_lastMousePosition to the restored position to prevent the jump on the next rotation start
                m_lastMousePosition = m_initialMousePosition;
            }
        }

        if (m_isRightMouseButtonPressed && (channelId == Movement::X || channelId == Movement::Y))
        {
            if (m_ignoreNextMovement)
            {
                m_ignoreNextMovement = false; // Reset the flag after ignoring one movement
                m_lastMousePosition = GetCurrentMousePosition(); // Update the last position to current to avoid the jump
                return; // Do not process this movement
            }

            AZ::Vector2 currentMousePosition = GetCurrentMousePosition();
            AZ::Vector2 mouseDelta = currentMousePosition - m_lastMousePosition;

            RotateCameraOnMouse(mouseDelta);

            const auto center = AZ::Vector2(0.5f, 0.5f);

            // Recenter the cursor to avoid edge constraints
            AzFramework::InputSystemCursorRequestBus::Event(
                AzFramework::InputDeviceMouse::Id, &AzFramework::InputSystemCursorRequests::SetSystemCursorPositionNormalized, center);

            // Then update m_lastMousePosition accordingly
            m_lastMousePosition = center;
        }
    }

    AZ::Vector2 FollowingCameraComponent::GetCurrentMousePosition()
    {
        AZ::Vector2 mousePosition(0.0f, 0.0f);

        // Query the input system for the current mouse position
        AzFramework::InputSystemCursorRequestBus::EventResult(
            mousePosition, AzFramework::InputDeviceMouse::Id, &AzFramework::InputSystemCursorRequests::GetSystemCursorPositionNormalized);

        return mousePosition;
    }

    void FollowingCameraComponent::RotateCameraOnMouse(const AZ::Vector2& mouseDelta)
    {
        // Convert mouse delta to rotation angles
        float yawRotation = -mouseDelta.GetX() * m_configuration.m_rotationSensitivity; // Inverted X for reversed yaw rotation
        float pitchRotation = -mouseDelta.GetY() * m_configuration.m_rotationSensitivity; // Inverted Y for pitch, adjust as needed

        // Fetch the current camera orientation from m_cameraOffset
        const AZ::Quaternion currentRotation = AZ::Quaternion::CreateFromMatrix4x4(m_cameraOffset);

        AZ::Quaternion yawQuat = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), yawRotation);

        // Apply yaw rotation to current camera orientation
        AZ::Quaternion tempRotation = yawQuat * currentRotation;

        // Calculate pitch rotation around the camera's local X-axis after applying yaw
        // This ensures that pitch adjustments are made relative to the camera's adjusted orientation
        const AZ::Vector3 localXAxis = tempRotation.TransformVector(AZ::Vector3::CreateAxisX());
        const auto pitchQuat = AZ::Quaternion::CreateFromAxisAngle(localXAxis, pitchRotation);

        // Apply pitch rotation
        auto newRotation = pitchQuat * tempRotation;
        newRotation.Normalize(); // Normalize the quaternion to prevent drift and ensure no numerical instability

        // Update the camera's offset matrix with the new orientation, keeping the position unchanged
        const AZ::Vector3 currentPosition = m_cameraOffset.GetTranslation();
        m_cameraOffset = AZ::Matrix4x4::CreateFromQuaternionAndTranslation(newRotation, currentPosition);
    }

    void FollowingCameraComponent::MoveCameraOnKeys(const AzFramework::InputChannelId& channelId)
    {
        auto localCamPoseChange = AZ::Vector3::CreateZero();

        // Check each key state and adjust the camera pose change accordingly
        if (channelId == Key::AlphanumericW)
        {
            localCamPoseChange += AZ::Vector3::CreateAxisY() * m_configuration.m_translationSpeed;
        }
        if (channelId == Key::AlphanumericS)
        {
            localCamPoseChange -= AZ::Vector3::CreateAxisY() * m_configuration.m_translationSpeed;
        }
        if (channelId == Key::AlphanumericA)
        {
            localCamPoseChange -= AZ::Vector3::CreateAxisX() * m_configuration.m_translationSpeed;
        }
        if (channelId == Key::AlphanumericD)
        {
            localCamPoseChange += AZ::Vector3::CreateAxisX() * m_configuration.m_translationSpeed;
        }

        const auto localCamPoseChangeMatrix = AZ::Matrix4x4::CreateTranslation(localCamPoseChange);
        m_cameraOffset = m_cameraOffset * localCamPoseChangeMatrix;
    }

    void FollowingCameraComponent::RotateCameraOnKeys(const AzFramework::InputChannelId& channelId)
    {
        float yawRotation = 0.0f; // Initialize yaw rotation to 0

        if (channelId == Key::AlphanumericQ)
        {
            yawRotation = m_configuration.m_rotationSpeed; // Rotate left
        }
        else if (channelId == Key::AlphanumericE)
        {
            yawRotation = -m_configuration.m_rotationSpeed; // Rotate right
        }

        if (yawRotation != 0.0f) // Only proceed if there's a rotation to apply
        {
            // Fetch the current camera orientation from m_cameraOffset
            const AZ::Quaternion currentRotation = AZ::Quaternion::CreateFromMatrix4x4(m_cameraOffset);

            const AZ::Quaternion yawQuat = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(), yawRotation);

            // Apply yaw rotation to current camera orientation
            const AZ::Quaternion newRotation = yawQuat * currentRotation;

            // Update the camera's offset matrix with the new orientation, keeping the position unchanged
            AZ::Vector3 currentPosition = m_cameraOffset.GetTranslation();
            m_cameraOffset = AZ::Matrix4x4::CreateFromQuaternionAndTranslation(newRotation, currentPosition);
        }
    }

    void FollowingCameraComponent::KeyboardEvent(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputChannelId& channelId = inputChannel.GetInputChannelId();

        if (m_moveKeys.find(channelId) != m_moveKeys.end())
        {
            MoveCameraOnKeys(channelId);
        }
        else if (m_rotateKeys.find(channelId) != m_rotateKeys.end())
        {
            RotateCameraOnKeys(channelId);
        }

        // Handle view switching with numeric keys
        if (auto it = KeysToView.find(channelId); it != KeysToView.end())
        {
            if (int viewId = it->second; viewId < m_configuration.m_predefinedViews.size())
            {
                m_currentView = m_configuration.m_predefinedViews[viewId];
                m_lastTranslationsBuffer.clear();
                m_lastRotationsBuffer.clear();

                m_cameraOffset = GetEntityLocalPose(m_currentView); // Reset camera offset to the view's pose
            }
        }
    }

    AZ::Matrix4x4 FollowingCameraComponent::GetEntityLocalPose(AZ::EntityId entityId)
    {
        if (!entityId.IsValid())
        {
            return AZ::Matrix4x4::CreateIdentity();
        }

        // Get the entity's local transform
        AZ::Transform target_local_transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(target_local_transform, entityId, &AZ::TransformBus::Events::GetLocalTM);

        AZ::Matrix4x4 matrix = AZ::Matrix4x4::CreateFromTransform(target_local_transform);
        return matrix;
    }
} // namespace ROS2
