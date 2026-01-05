/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * ============================================================================
 * O3DE REFLECT - BEFORE AND AFTER COMPARISON
 * ============================================================================
 *
 * This file demonstrates the dramatic reduction in boilerplate when using
 * O3DEReflect compared to the traditional O3DE approach.
 *
 * ============================================================================
 * TRADITIONAL O3DE APPROACH (BEFORE) - ~150 lines
 * ============================================================================
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


// SOURCE FILE - PlayerMovementComponent.cpp
// ==========================================

/*
namespace TraditionalExample
{
    // MUST MANUALLY WRITE THE ENTIRE REFLECT FUNCTION
    void PlayerMovementComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PlayerMovementComponent, AZ::Component>()
                ->Version(1)
                ->Field("MoveSpeed", &PlayerMovementComponent::m_moveSpeed)
                ->Field("SprintMultiplier", &PlayerMovementComponent::m_sprintMultiplier)
                ->Field("JumpForce", &PlayerMovementComponent::m_jumpForce)
                ->Field("MaxJumps", &PlayerMovementComponent::m_maxJumps)
                ->Field("GravityMultiplier", &PlayerMovementComponent::m_gravityMultiplier)
                ->Field("IsGrounded", &PlayerMovementComponent::m_isGrounded)
                ->Field("CurrentVelocity", &PlayerMovementComponent::m_currentVelocity)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<PlayerMovementComponent>(
                    "Player Movement", 
                    "Handles player character movement with configurable speed, jumping, and physics")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Gameplay/Movement")
                        ->Attribute(AZ::Edit::Attributes::Icon, "Icons/Components/Movement.svg")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    
                    // MUST MANUALLY CONFIGURE EACH PROPERTY
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Movement")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PlayerMovementComponent::m_moveSpeed, 
                        "Move Speed", "Base movement speed in meters per second")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 100.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, "m/s")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PlayerMovementComponent::m_sprintMultiplier,
                        "Sprint Multiplier", "Speed multiplier when sprinting")
                        ->Attribute(AZ::Edit::Attributes::Min, 1.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 5.0f)
                    
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Jumping")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PlayerMovementComponent::m_jumpForce,
                        "Jump Force", "Upward force applied when jumping")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 2000.0f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, "N")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PlayerMovementComponent::m_maxJumps,
                        "Max Jumps", "Maximum number of jumps (1 = no double jump)")
                        ->Attribute(AZ::Edit::Attributes::Min, 1)
                        ->Attribute(AZ::Edit::Attributes::Max, 5)
                    
                    ->ClassElement(AZ::Edit::ClassElements::Group, "Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PlayerMovementComponent::m_gravityMultiplier,
                        "Gravity Multiplier", "Multiplier for gravity effect on this character")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Max, 5.0f)
                    
                    ->ClassElement(AZ::Edit::ClassElements::Group, "State")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PlayerMovementComponent::m_isGrounded,
                        "Is Grounded", "Whether the character is currently on the ground")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PlayerMovementComponent::m_currentVelocity,
                        "Current Velocity", "Current velocity of the character")
                        ->Attribute(AZ::Edit::Attributes::ReadOnly, true)
                    ;
            }
        }

        // MUST MANUALLY WRITE BEHAVIOR CONTEXT FOR SCRIPTING
        if (auto* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<PlayerMovementComponent>("PlayerMovement")
                ->Attribute(AZ::Script::Attributes::Category, "Gameplay/Movement")
                ->Property("moveSpeed",
                    [](PlayerMovementComponent* self) { return self->GetMoveSpeed(); },
                    [](PlayerMovementComponent* self, float value) { self->SetMoveSpeed(value); })
                ->Property("sprintMultiplier",
                    [](PlayerMovementComponent* self) { return self->GetSprintMultiplier(); },
                    [](PlayerMovementComponent* self, float value) { self->SetSprintMultiplier(value); })
                ->Property("jumpForce",
                    [](PlayerMovementComponent* self) { return self->GetJumpForce(); },
                    [](PlayerMovementComponent* self, float value) { self->SetJumpForce(value); })
                ->Property("maxJumps",
                    [](PlayerMovementComponent* self) { return self->GetMaxJumps(); },
                    [](PlayerMovementComponent* self, int value) { self->SetMaxJumps(value); })
                ->Property("isGrounded",
                    [](PlayerMovementComponent* self) { return self->GetIsGrounded(); },
                    nullptr)
                ->Property("currentVelocity",
                    [](PlayerMovementComponent* self) { return self->GetCurrentVelocity(); },
                    nullptr)
                ->Method("Jump", &PlayerMovementComponent::Jump)
                ->Method("Move", &PlayerMovementComponent::Move)
                ->Method("SetSprinting", &PlayerMovementComponent::SetSprinting)
                ->Method("GetCurrentSpeed", &PlayerMovementComponent::GetCurrentSpeed)
                ->Method("Teleport", &PlayerMovementComponent::Teleport)
                ;
        }
    }

    // MUST MANUALLY WRITE ALL SERVICE FUNCTIONS
    void PlayerMovementComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("PlayerMovementService"));
    }

    void PlayerMovementComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        (void)incompatible;
    }

    void PlayerMovementComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void PlayerMovementComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("PhysicsService"));
    }

    void PlayerMovementComponent::Init() { }
    void PlayerMovementComponent::Activate() { }
    void PlayerMovementComponent::Deactivate() { }
    void PlayerMovementComponent::Jump() { }
    void PlayerMovementComponent::Move(const AZ::Vector3& direction, float deltaTime) { }
    void PlayerMovementComponent::SetSprinting(bool sprinting) { }
    float PlayerMovementComponent::GetCurrentSpeed() const { return m_moveSpeed; }
    void PlayerMovementComponent::Teleport(const AZ::Vector3& position) { }

} // namespace TraditionalExample
*/


/*
 * ============================================================================
 * O3DE REFLECT APPROACH (AFTER) - ~80 lines of XML (see Examples folder)
 * ============================================================================
 *
 * With O3DEReflect, the entire component is defined in a clean XML file:
 * 
 *   Examples/PlayerMovementComponent.O3DEReflect.xml
 * 
 * The code generator produces:
 *   - PlayerMovementComponent.AutoReflect.h  (complete header)
 *   - PlayerMovementComponent.AutoReflect.cpp (complete implementation)
 *
 * BENEFITS:
 * 
 * 1. LESS CODE TO WRITE
 *    - Traditional: ~200+ lines of C++ boilerplate
 *    - O3DEReflect: ~80 lines of declarative XML
 * 
 * 2. LESS ERROR-PRONE
 *    - No manual serialization/deserialization code
 *    - No manually matching property names in 3 places
 *    - Automatic UUID generation prevents conflicts
 * 
 * 3. FASTER ITERATION
 *    - Change XML, rebuild - code regenerated automatically
 *    - Add properties without touching reflection code
 *
 * 4. CONSISTENT PATTERNS
 *    - All components follow the same structure
 *    - Automatic getter/setter generation
 *    - Consistent script bindings
 *
 * 5. DOCUMENTATION BUILT-IN
 *    - Tooltips defined once in XML
 *    - Appear in Editor AND in code comments
 *
 * ============================================================================
 * COMPARISON SUMMARY
 * ============================================================================
 *
 * | Aspect                    | Traditional | O3DEReflect |
 * |---------------------------|-------------|-------------|
 * | Lines of code             | ~200+       | ~80 XML     |
 * | Manual Reflect() function | Yes         | Generated   |
 * | Manual service functions  | Yes         | Generated   |
 * | Manual getters/setters    | Yes         | Generated   |
 * | Manual script bindings    | Yes         | Generated   |
 * | Error-prone               | High        | Low         |
 * | Learning curve            | Steep       | Gradual     |
 *
 * ============================================================================
 */
