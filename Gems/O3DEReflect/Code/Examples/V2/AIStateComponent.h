/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * V2 Example: AI State Component using header-macro parsing
 * 
 * Demonstrates a more complex component with enums and state management.
 */

#pragma once

#include <O3DEReflect/O3DEReflectMacros.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

// ============================================================================
// AI State Enum
// ============================================================================

O3DE_ENUM(BlueprintType,
    Category = "AI",
    Description = "Possible states for AI behavior"
)
enum class AIState : uint8_t
{
    O3DE_ENUM_VALUE(Idle, DisplayName = "Idle", Description = "AI is standing still, not engaged")
    Idle = 0,

    O3DE_ENUM_VALUE(Patrol, DisplayName = "Patrolling", Description = "AI is following a patrol route")
    Patrol,

    O3DE_ENUM_VALUE(Alert, DisplayName = "Alert", Description = "AI has detected something suspicious")
    Alert,

    O3DE_ENUM_VALUE(Chase, DisplayName = "Chasing", Description = "AI is pursuing a target")
    Chase,

    O3DE_ENUM_VALUE(Combat, DisplayName = "In Combat", Description = "AI is actively fighting")
    Combat,

    O3DE_ENUM_VALUE(Flee, DisplayName = "Fleeing", Description = "AI is running away")
    Flee,

    O3DE_ENUM_VALUE(Dead, DisplayName = "Dead", Description = "AI is dead/disabled")
    Dead
};

// ============================================================================
// AI Behavior Flags
// ============================================================================

O3DE_ENUM(BlueprintType, Flags,
    Category = "AI",
    Description = "Behavior modifiers for AI"
)
enum class AIBehaviorFlags : uint32_t
{
    O3DE_ENUM_VALUE(None, DisplayName = "None")
    None = 0,

    O3DE_ENUM_VALUE(CanPatrol, DisplayName = "Can Patrol")
    CanPatrol = 1 << 0,

    O3DE_ENUM_VALUE(CanChase, DisplayName = "Can Chase")
    CanChase = 1 << 1,

    O3DE_ENUM_VALUE(CanFlee, DisplayName = "Can Flee")
    CanFlee = 1 << 2,

    O3DE_ENUM_VALUE(CanCallForHelp, DisplayName = "Can Call For Help")
    CanCallForHelp = 1 << 3,

    O3DE_ENUM_VALUE(Aggressive, DisplayName = "Aggressive (attacks on sight)")
    Aggressive = 1 << 4,

    O3DE_ENUM_VALUE(Cowardly, DisplayName = "Cowardly (flees when hurt)")
    Cowardly = 1 << 5
};

// ============================================================================
// Patrol Point Struct
// ============================================================================

O3DE_STRUCT(BlueprintType,
    Category = "AI",
    Description = "A waypoint in a patrol route"
)
struct PatrolPoint
{
    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Patrol",
        Tooltip = "World position of this patrol point"
    )
    AZ::Vector3 position = AZ::Vector3::CreateZero();

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Patrol",
        Tooltip = "How long to wait at this point (seconds)",
        Min = 0.0f, Max = 60.0f
    )
    float waitTime = 2.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Patrol",
        Tooltip = "Should the AI look around at this point?"
    )
    bool lookAround = false;
};

// ============================================================================
// AI State Component
// ============================================================================

O3DE_CLASS(
    Category = "AI/Core",
    Description = "Manages AI state machine and behavior",
    Icon = "Icons/Components/AI.svg"
)
class AIStateComponent : public AZ::Component
{
    O3DE_GENERATED_BODY()

    O3DE_COMPONENT(AIStateComponent, "{B2C3D4E5-F6A7-8901-BCDE-F23456789ABC}",
        ProvidesServices = "AIStateService",
        RequiresServices = "TransformService",
        IncompatibleServices = "PlayerControllerService"
    )

public:
    // ========================================================================
    // State Properties
    // ========================================================================

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "State",
        Tooltip = "Current AI state"
    )
    AIState m_currentState = AIState::Idle;

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "State",
        Tooltip = "Previous AI state (for transition logic)"
    )
    AIState m_previousState = AIState::Idle;

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "State",
        Tooltip = "Time spent in current state (seconds)"
    )
    float m_timeInState = 0.0f;

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "State",
        Tooltip = "Current target entity (if any)"
    )
    AZ::EntityId m_currentTarget;

    // ========================================================================
    // Configuration Properties
    // ========================================================================

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Detection",
        Tooltip = "How far the AI can see",
        Min = 1.0f, Max = 100.0f, Suffix = "m"
    )
    float m_sightRange = 20.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Detection",
        Tooltip = "Field of view angle (degrees)",
        Min = 30.0f, Max = 360.0f, Suffix = "°"
    )
    float m_fieldOfView = 120.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Detection",
        Tooltip = "How far the AI can hear",
        Min = 1.0f, Max = 50.0f, Suffix = "m"
    )
    float m_hearingRange = 15.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Behavior",
        Tooltip = "Behavior flags controlling AI capabilities"
    )
    AIBehaviorFlags m_behaviorFlags = AIBehaviorFlags::CanPatrol;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Combat",
        Tooltip = "Preferred combat distance",
        Min = 1.0f, Max = 30.0f, Suffix = "m"
    )
    float m_preferredCombatRange = 5.0f;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Combat",
        Tooltip = "Health threshold to trigger flee (0-1)",
        Min = 0.0f, Max = 1.0f
    )
    float m_fleeHealthThreshold = 0.2f;

    // ========================================================================
    // Patrol Properties
    // ========================================================================

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Patrol",
        Tooltip = "Patrol waypoints"
    )
    AZStd::vector<PatrolPoint> m_patrolPoints;

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly,
        Category = "Patrol",
        Tooltip = "Current patrol point index"
    )
    int m_currentPatrolIndex = 0;

    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Patrol",
        Tooltip = "Should patrol loop back to start?"
    )
    bool m_loopPatrol = true;

    // ========================================================================
    // State Control Functions
    // ========================================================================

    O3DE_FUNCTION(BlueprintCallable,
        Category = "State",
        Tooltip = "Force transition to a new state"
    )
    void SetState(AIState newState);

    O3DE_FUNCTION(BlueprintPure, BlueprintCallable,
        Category = "State",
        Tooltip = "Check if AI is in a specific state"
    )
    bool IsInState(AIState state) const;

    O3DE_FUNCTION(BlueprintCallable,
        Category = "State",
        Tooltip = "Set the current target entity"
    )
    void SetTarget(AZ::EntityId target);

    O3DE_FUNCTION(BlueprintCallable,
        Category = "State",
        Tooltip = "Clear the current target"
    )
    void ClearTarget();

    // ========================================================================
    // Detection Functions
    // ========================================================================

    O3DE_FUNCTION(BlueprintPure, BlueprintCallable,
        Category = "Detection",
        Tooltip = "Check if AI can see a specific entity"
    )
    bool CanSeeEntity(AZ::EntityId entity) const;

    O3DE_FUNCTION(BlueprintPure, BlueprintCallable,
        Category = "Detection",
        Tooltip = "Check if AI can hear a position"
    )
    bool CanHearPosition(const AZ::Vector3& position, float noiseLevel) const;

    O3DE_FUNCTION(BlueprintCallable,
        Category = "Detection",
        Tooltip = "Scan for nearby threats, returns closest threat"
    )
    AZ::EntityId ScanForThreats();

    // ========================================================================
    // Patrol Functions
    // ========================================================================

    O3DE_FUNCTION(BlueprintCallable,
        Category = "Patrol",
        Tooltip = "Add a patrol point at runtime"
    )
    void AddPatrolPoint(const PatrolPoint& point);

    O3DE_FUNCTION(BlueprintCallable,
        Category = "Patrol",
        Tooltip = "Clear all patrol points"
    )
    void ClearPatrolPoints();

    O3DE_FUNCTION(BlueprintPure, BlueprintCallable,
        Category = "Patrol",
        Tooltip = "Get the next patrol point position"
    )
    AZ::Vector3 GetNextPatrolPosition() const;

    // ========================================================================
    // Debug Functions
    // ========================================================================

    O3DE_FUNCTION(CallInEditor,
        Category = "Debug",
        Tooltip = "Draw detection ranges in editor"
    )
    void DrawDebugVisualization();

    O3DE_FUNCTION(CallInEditor,
        Category = "Debug",
        Tooltip = "Reset AI to initial state"
    )
    void ResetAI();

protected:
    void Activate() override;
    void Deactivate() override;

private:
    void UpdateStateMachine(float deltaTime);
    void OnStateEnter(AIState state);
    void OnStateExit(AIState state);
};
