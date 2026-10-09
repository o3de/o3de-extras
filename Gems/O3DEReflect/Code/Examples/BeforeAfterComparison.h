/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Math/Vector3.h>

namespace TraditionalExample
{
    // HEADER FILE - PlayerMovementComponent.h
    // ========================================
    
    class PlayerMovementComponent
        : public AZ::Component
    {
    public:
        AZ_COMPONENT(PlayerMovementComponent, "{A1B2C3D4-E5F6-7890-ABCD-123456789ABC}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        PlayerMovementComponent() = default;
        ~PlayerMovementComponent() override = default;

        // Getters and Setters - MUST WRITE MANUALLY
        float GetMoveSpeed() const { return m_moveSpeed; }
        void SetMoveSpeed(float value) { m_moveSpeed = value; }
        
        float GetSprintMultiplier() const { return m_sprintMultiplier; }
        void SetSprintMultiplier(float value) { m_sprintMultiplier = value; }
        
        float GetJumpForce() const { return m_jumpForce; }
        void SetJumpForce(float value) { m_jumpForce = value; }
        
        int GetMaxJumps() const { return m_maxJumps; }
        void SetMaxJumps(int value) { m_maxJumps = value; }
        
        bool GetIsGrounded() const { return m_isGrounded; }
        const AZ::Vector3& GetCurrentVelocity() const { return m_currentVelocity; }

        void Jump();
        void Move(const AZ::Vector3& direction, float deltaTime);
        void SetSprinting(bool sprinting);
        float GetCurrentSpeed() const;
        void Teleport(const AZ::Vector3& position);

    protected:
        void Init() override;
        void Activate() override;
        void Deactivate() override;

    private:
        // MUST MANUALLY DECLARE ALL MEMBERS
        float m_moveSpeed = 5.0f;
        float m_sprintMultiplier = 2.0f;
        float m_jumpForce = 400.0f;
        int m_maxJumps = 2;
        float m_gravityMultiplier = 1.0f;
        bool m_isGrounded = false;
        AZ::Vector3 m_currentVelocity = AZ::Vector3::CreateZero();
    };

} // namespace TraditionalExample


