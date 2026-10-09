/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * V2 Example: DamageInfo struct using header-macro parsing
 * 
 * Demonstrates O3DE_STRUCT for data-only structures.
 */

#pragma once

#include <O3DEReflect/O3DEReflectMacros.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/string/string.h>

// Forward declare the enum
enum class DamageType : uint8_t;

// ============================================================================
// DamageInfo Struct - Data container for damage events
// ============================================================================

O3DE_STRUCT(ScriptType,
    Category = "Combat",
    Description = "Contains all information about a damage event"
)
struct DamageInfo
{
    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Damage",
        Tooltip = "Base damage amount before modifiers",
        Min = 0.0f
    )
    float baseDamage = 0.0f;

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Damage",
        Tooltip = "Damage multiplier (crits, weaknesses, etc.)",
        Min = 0.0f, Max = 10.0f
    )
    float damageMultiplier = 1.0f;

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Source",
        Tooltip = "Entity that caused the damage"
    )
    AZ::EntityId instigator;

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Source",
        Tooltip = "Entity that directly dealt the damage (e.g., projectile)"
    )
    AZ::EntityId damageCauser;

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Location",
        Tooltip = "World position where damage was applied"
    )
    AZ::Vector3 hitLocation = AZ::Vector3::CreateZero();

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Location",
        Tooltip = "Direction the damage came from"
    )
    AZ::Vector3 hitDirection = AZ::Vector3::CreateZero();

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Location",
        Tooltip = "Name of the bone/hitbox that was hit"
    )
    AZStd::string hitBoneName;

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Damage",
        Tooltip = "Type of damage (for resistance calculations)"
    )
    DamageType damageType;

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Flags",
        Tooltip = "Was this a critical hit?"
    )
    bool isCriticalHit = false;

    O3DE_PROPERTY(EditAnywhere, ScriptReadWrite,
        Category = "Flags",
        Tooltip = "Should this damage be displayed in UI?"
    )
    bool showDamageNumbers = true;

    // Helper methods (exposed to scripting)
    O3DE_FUNCTION(ScriptPure, ScriptCallable,
        Category = "Damage",
        Tooltip = "Calculate final damage after multipliers"
    )
    float GetFinalDamage() const
    {
        return baseDamage * damageMultiplier;
    }

    O3DE_FUNCTION(ScriptPure, ScriptCallable,
        Category = "Damage",
        Tooltip = "Check if damage is lethal (would reduce health to 0)"
    )
    bool IsLethal(float currentHealth) const
    {
        return GetFinalDamage() >= currentHealth;
    }
};

// ============================================================================
// DamageType Enum
// ============================================================================

O3DE_ENUM(ScriptType,
    Category = "Combat",
    Description = "Types of damage for resistance calculations"
)
enum class DamageType : uint8_t
{
    O3DE_ENUM_VALUE(Physical, DisplayName = "Physical Damage")
    Physical = 0,

    O3DE_ENUM_VALUE(Fire, DisplayName = "Fire Damage")
    Fire,

    O3DE_ENUM_VALUE(Ice, DisplayName = "Ice/Cold Damage")
    Ice,

    O3DE_ENUM_VALUE(Electric, DisplayName = "Electric/Lightning Damage")
    Electric,

    O3DE_ENUM_VALUE(Poison, DisplayName = "Poison/Toxic Damage")
    Poison,

    O3DE_ENUM_VALUE(Magic, DisplayName = "Magic/Arcane Damage")
    Magic,

    O3DE_ENUM_VALUE(True, DisplayName = "True Damage (ignores resistances)")
    True
};

// ============================================================================
// DamageFlags Enum (bitmask)
// ============================================================================

O3DE_ENUM(ScriptType, Flags,
    Category = "Combat",
    Description = "Flags that modify damage behavior"
)
enum class DamageFlags : uint32_t
{
    O3DE_ENUM_VALUE(None, DisplayName = "No Flags")
    None = 0,

    O3DE_ENUM_VALUE(IgnoreArmor, DisplayName = "Ignore Armor")
    IgnoreArmor = 1 << 0,

    O3DE_ENUM_VALUE(IgnoreShield, DisplayName = "Ignore Shield")
    IgnoreShield = 1 << 1,

    O3DE_ENUM_VALUE(CanCrit, DisplayName = "Can Critical Hit")
    CanCrit = 1 << 2,

    O3DE_ENUM_VALUE(CanBeBlocked, DisplayName = "Can Be Blocked")
    CanBeBlocked = 1 << 3,

    O3DE_ENUM_VALUE(AppliesKnockback, DisplayName = "Applies Knockback")
    AppliesKnockback = 1 << 4,

    O3DE_ENUM_VALUE(AppliesStagger, DisplayName = "Applies Stagger")
    AppliesStagger = 1 << 5,

    O3DE_ENUM_VALUE(OverTime, DisplayName = "Damage Over Time")
    OverTime = 1 << 6,

    O3DE_ENUM_VALUE(AreaOfEffect, DisplayName = "Area of Effect")
    AreaOfEffect = 1 << 7
};

// Bitwise operators for DamageFlags
inline DamageFlags operator|(DamageFlags a, DamageFlags b)
{
    return static_cast<DamageFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline DamageFlags operator&(DamageFlags a, DamageFlags b)
{
    return static_cast<DamageFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline bool HasFlag(DamageFlags flags, DamageFlags flag)
{
    return (flags & flag) == flag;
}
