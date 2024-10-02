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
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Input/Events/InputChannelEventListener.h>

namespace ROS2
{
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
        void OnKeyboardEvent(const AzFramework::InputChannel& inputChannel);

        //! Remove the tilt from the transform.
        //! The tilt is removed by projecting the transform basis of rotation matrix to the ground plane.
        //! @param transform The transform to remove the tilt.
        //! @return The transform without tilt.
        AZ::Transform RemoveTiltFromTransform(AZ::Transform transform);

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
        AZStd::deque<AZStd::pair<AZ::Quaternion, float>> m_lastRotationsBuffer;

        float m_rotationOffset = 0.0f; //!< The rotation change from the input.
        float m_opticalAxisTranslation = 0.0f; //!< The zoom change from the input.
        AZ::EntityId m_currentView; //!< Current used view point.

        FollowingCameraConfiguration m_configuration; //!< The configuration of the following camera.
    };
} // namespace ROS2
