/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * V2 Example: PlayerMovementComponent using header-macro parsing
 * 
 * This file demonstrates the Unreal-style reflection syntax.
 * The O3DEReflect generator will parse these macros and generate:
 *   - PlayerMovementComponent.generated.h (included via O3DE_GENERATED_BODY)
 *   - PlayerMovementComponent.generated.cpp (reflection implementation)
 *
 * Compare this to the equivalent XML definition in:
 *   ../PlayerMovementComponent.O3DEReflect.xml
 */

#pragma once

#include <O3DEReflect/O3DEReflectMacros.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Component/EntityId.h>

// ============================================================================
// V2 Header-Macro Style - Clean, Unreal-like syntax!
// ============================================================================

O3DE_CLASS(
    Category = "Gameplay/Movement",
    Description = "Handles player movement with physics-based locomotion"
)
class PlayerMovementComponent : public AZ::Component
{
    // This macro includes the generated declarations and Reflect() signature
    O3DE_GENERATED_BODY()

    // Explicit UUID (optional - will auto-generate deterministic UUID if omitted)
    O3DE_COMPONENT(PlayerMovementComponent, "{A1B2C3D4-E5F6-7890-ABCD-EF1234567890}",
        ProvidesServices = "PlayerMovementService",
        RequiresServices = "TransformService",
        IncompatibleServices = "AIMovementService"
    )

public:
    // ========================================================================
    // Movement Properties
    // ========================================================================

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Movement|Speed",
        Tooltip = "Maximum movement speed in meters per second",
        DisplayName = "Max Speed",
        Min = 0.0f, Max = 100.0f, UIMax = 50.0f,
        Suffix = "m/s"
    )
    float m_maxSpeed = 10.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Movement|Speed",
        Tooltip = "Acceleration rate",
        Min = 0.0f, Max = 500.0f
    )
    float m_acceleration = 50.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Movement|Speed",
        Tooltip = "Deceleration rate when not moving",
        Min = 0.0f, Max = 500.0f
    )
    float m_deceleration = 40.0f;

    // ========================================================================
    // Jump Properties
    // ========================================================================

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Movement|Jump",
        Tooltip = "Initial velocity when jumping",
        Min = 0.0f, Max = 50.0f
    )
    float m_jumpVelocity = 8.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Movement|Jump",
        Tooltip = "Number of jumps allowed before landing",
        Min = 1, Max = 5
    )
    int m_maxJumps = 2;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadOnly,
        Category = "Movement|Jump",
        Tooltip = "Custom gravity multiplier"
    )
    float m_gravityMultiplier = 1.0f;

    // ========================================================================
    // State Properties (runtime, visible but not editable)
    // ========================================================================

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "State",
        Tooltip = "Current velocity vector"
    )
    AZ::Vector3 m_currentVelocity = AZ::Vector3::CreateZero();

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "State",
        Tooltip = "Is the player currently on the ground?"
    )
    bool m_isGrounded = false;

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "State",
        Tooltip = "Remaining jumps available"
    )
    int m_jumpsRemaining = 2;

    // ========================================================================
    // Exposed Functions
    // ========================================================================

    O3DE_FUNCTION(BlueprintCallable,
        Category = "Movement",
        Tooltip = "Apply movement input direction",
        DisplayName = "Move"
    )
    void Move(const AZ::Vector3& direction);

    O3DE_FUNCTION(BlueprintCallable,
        Category = "Movement",
        Tooltip = "Attempt to jump, returns true if successful"
    )
    bool Jump();

    O3DE_FUNCTION(BlueprintCallable,
        Category = "Movement",
        Tooltip = "Stop all movement immediately"
    )
    void StopMovement();

    O3DE_FUNCTION(BlueprintPure, BlueprintCallable,
        Category = "State",
        Tooltip = "Get current speed (magnitude of velocity)"
    )
    float GetCurrentSpeed() const;

    O3DE_FUNCTION(BlueprintPure, BlueprintCallable,
        Category = "State",
        Tooltip = "Check if player is moving"
    )
    bool IsMoving() const;

    O3DE_FUNCTION(CallInEditor,
        Category = "Debug",
        Tooltip = "Reset player to spawn position"
    )
    void ResetToSpawn();

protected:
    // Component interface (these are manually implemented)
    void Activate() override;
    void Deactivate() override;

private:
    // Private implementation details (not reflected)
    void UpdateMovement(float deltaTime);
    void ApplyGravity(float deltaTime);
    AZ::Vector3 m_pendingInput = AZ::Vector3::CreateZero();
};

// In PlayerMovementComponent.cpp, you would add:
// O3DE_IMPLEMENT_REFLECT(PlayerMovementComponent)
