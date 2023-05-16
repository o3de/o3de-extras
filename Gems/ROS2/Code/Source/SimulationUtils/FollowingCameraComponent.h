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
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>

namespace ROS2
{
    //! The component used for camera following robots.
    //! It allows to switch between different camera that can be attached to entities, and to control the camera with the keyboard.
    //! It allows to offset angle around parent's Z axis, and to offset the distance from parent.
    //! More over it is smoothing movement of the camera.
    //! The theoretical background for handling quaternions as tangent vectors can be found in the paper:
    //! "A micro Lie theory for state estimation in robotics" by Joan Solà, J. M. Porta, and Alberto Sanfeliu.
    class FollowingCameraComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public AzFramework::InputChannelEventListener
    {
    public:
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void Reflect(AZ::ReflectContext* reflection);

        AZ_COMPONENT(FollowingCameraComponent, "{6a21768a-f327-11ed-a05b-0242ac120003}", AZ::Component);

        // AZ::Component
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // AzFramework::InputChannelEventListener overrides
        bool OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel) override;

    private:
        void OnKeyboardEvent(const AzFramework::InputChannel& inputChannel);

        //! Compute weighted average of the vectors in the buffer.
        //! @param buffer The buffer to compute the average.
        //! @return The average vector.
        AZ::Vector3 AverageVector(const AZStd::deque<AZStd::pair<AZ::Vector3, float>>& buffer) const;

        //! Compute weighted average of translation in the buffer.
        //! @return The average translation.
        AZ::Vector3 SmoothTranslation() const;

        //! Compute weighted average of rotation in the buffer.
        //! @return The average rotation.
        AZ::Quaternion SmoothRotation() const;

        //! Cache the transform in smoothing buffer.
        //! @param transform The transform to cache.
        //! @param deltaTime The time between the last frame and the current frame.
        void CacheTransform(const AZ::Transform& transform, float deltaTime);

        //! The smoothing buffer for translation, the first element is the translation, the second element is the weight.
        AZStd::deque<AZStd::pair<AZ::Vector3, float>> m_lastTranslationsBuffer;

        //! The smoothing buffer for rotation, the first element is the tangential vector, the second element is the weight.
        AZStd::deque<AZStd::pair<AZ::Vector3, float>> m_lastRotationsBuffer;

        //! The last translation, used to compute determine the incremental rotation.
        AZ::Quaternion m_lastRotation;

        //! Predefined view points.
        AZStd::vector<AZ::EntityId> m_predefinedViews;

        //! Default view point to active after activation.
        int m_defaultView{ 0 };

        //! Current used view point.
        AZ::EntityId m_currentView;

        //! The rotation and zoom change from the input.
        float m_rotationOffset = 0.0f;
        //! The translation offset from the defined view.
        float m_opticalAxisTranslation = 0.0f;

        //! The length of the smoothing buffer.
        int m_smoothingBuffer = 30;
        float m_zoomSpeed = 0.06f;
        float m_rotationSpeed = 0.05f;

        const float m_opticalAxisTranslationMin = 0.0f;
    };
} // namespace ROS2
