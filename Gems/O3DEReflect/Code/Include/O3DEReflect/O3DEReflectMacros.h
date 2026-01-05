/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * O3DE Reflect System - Unreal-style reflection macros for simplified component/struct/enum creation
 * 
 *  Header-Macro Parsing
 * ========================
 * These macros are designed to be parsed by the O3DEReflect code generator.
 * They expand to empty or minimal code at compile time, while providing
 * metadata that the generator uses to create reflection code.
 *
 * Usage:
 *   O3DE_CLASS(Category="Gameplay", Description="My awesome component")
 *   class MyComponent : public AZ::Component {
 *       O3DE_GENERATED_BODY()
 *       
 *       O3DE_PROPERTY(EditAnywhere, Category="Movement", Tooltip="Speed in m/s", Min=0.0f, Max=100.0f)
 *       float m_speed = 10.0f;
 *
 *       O3DE_FUNCTION(BlueprintCallable, Category="Actions")
 *       void DoSomething();
 *   };
 */

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>

// ============================================================================
// MACRO IMPLEMENTATION STRATEGY
// ============================================================================
//
// These macros serve dual purposes:
// 1. At compile time: They expand to minimal/empty code
// 2. At parse time: The O3DEReflect generator extracts metadata from them
//
// The generator parses the source file and looks for these macro patterns,
// then generates the appropriate reflection code (.generated.h/.generated.cpp)
//
// ============================================================================

// ============================================================================
// Property Visibility Specifiers (like Unreal's UPROPERTY specifiers)
// ============================================================================
// These are used as bare tokens within O3DE_PROPERTY()
//
// Editor visibility - controls how property appears in Entity Inspector:
//   EditAnywhere       - Editable in editor on instances and archetypes
//   EditDefaultsOnly   - Editable only on archetypes/prefabs
//   EditInstanceOnly   - Editable only on instances
//   VisibleAnywhere    - Visible but not editable
//   VisibleDefaultsOnly - Visible only on defaults
//   VisibleInstanceOnly - Visible only on instances
//
// Script visibility - controls script access:
//   BlueprintReadWrite - Readable and writable from Lua/ScriptCanvas
//   BlueprintReadOnly  - Readable only from Lua/ScriptCanvas
//
// Note: These are parsed as string tokens, not as macro values

// ============================================================================
// O3DE_CLASS - Mark a class for reflection
// ============================================================================
// 
// Attributes:
//   Category     - Category in Add Component menu (e.g., "Gameplay/Movement")
//   Description  - Tooltip description for the class
//   Icon         - Path to icon file (e.g., "Icons/Components/MyComponent.svg")
//   HideInEditor - If true, component won't appear in Add Component menu
//   Abstract     - If true, class cannot be instantiated directly
//
// Example:
//   O3DE_CLASS(Category="Gameplay", Description="Handles player movement")
//   class PlayerMovementComponent : public AZ::Component { ... };
//
#define O3DE_CLASS(...) \
    /* This macro is parsed by O3DEReflect code generator */ \
    /* Attributes: __VA_ARGS__ */

// ============================================================================
// O3DE_COMPONENT - Declare component with UUID (use instead of AZ_COMPONENT)
// ============================================================================
//
// Automatically provides:
//   - AZ_COMPONENT_DECL macro expansion
//   - UUID generation (auto or explicit)
//   - Service declarations
//
// Attributes:
//   ProvidesServices      - Comma-separated service names this component provides
//   RequiresServices      - Services this component requires
//   IncompatibleServices  - Services this component is incompatible with
//   DependentServices     - Services that should activate before this component
//
// Example:
//   O3DE_COMPONENT(MyComponent, "{12345678-1234-1234-1234-123456789ABC}",
//       ProvidesServices="MyService",
//       RequiresServices="TransformService")
//
#define O3DE_COMPONENT(ClassName, ...) \
    AZ_COMPONENT_DECL(ClassName) \
    /* UUID and services parsed by O3DEReflect code generator */ \
    /* Attributes: __VA_ARGS__ */

// ============================================================================
// O3DE_PROPERTY - Mark a member for serialization and editor exposure
// ============================================================================
//
// Visibility Specifiers:
//   EditAnywhere       - Editable everywhere in editor
//   EditDefaultsOnly   - Editable only on prefabs/archetypes
//   EditInstanceOnly   - Editable only on instances
//   VisibleAnywhere    - Visible but read-only in editor
//   BlueprintReadWrite - Exposed to Lua/ScriptCanvas read+write
//   BlueprintReadOnly  - Exposed to Lua/ScriptCanvas read-only
//
// Metadata Attributes:
//   Category    - Property category in inspector (e.g., "Movement|Speed")
//   Tooltip     - Hover tooltip text
//   DisplayName - Override display name (default: variable name)
//   Min         - Minimum value for numeric types
//   Max         - Maximum value for numeric types
//   UIMin       - UI slider minimum (can differ from Min)
//   UIMax       - UI slider maximum (can differ from Max)
//   Suffix      - Unit suffix (e.g., "m/s", "degrees")
//   ChangeNotify - Function to call when value changes
//
// Examples:
//   O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite, Category="Movement", Tooltip="Speed in m/s")
//   float m_speed = 10.0f;
//
//   O3DE_PROPERTY(EditAnywhere, Category="Combat", Min=0, Max=100)
//   int m_health = 100;
//
//   O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly)
//   AZ::EntityId m_target;
//
#define O3DE_PROPERTY(...) \
    /* This macro is parsed by O3DEReflect code generator */ \
    /* Attributes: __VA_ARGS__ */

// ============================================================================
// O3DE_FUNCTION - Expose a function to reflection/scripting
// ============================================================================
//
// Specifiers:
//   BlueprintCallable   - Can be called from Lua/ScriptCanvas
//   BlueprintPure       - Pure function (no side effects, can be cached)
//   CallInEditor        - Can be invoked from editor buttons
//   Server              - Only runs on server (multiplayer)
//   Client              - Only runs on client (multiplayer)
//   NetMulticast        - Runs on all connected clients
//
// Attributes:
//   Category    - Category in script node palette
//   DisplayName - Override function display name
//   Tooltip     - Hover tooltip text
//
// Example:
//   O3DE_FUNCTION(BlueprintCallable, Category="Combat", Tooltip="Deals damage to target")
//   void DealDamage(float amount, AZ::EntityId target);
//
#define O3DE_FUNCTION(...) \
    /* This macro is parsed by O3DEReflect code generator */ \
    /* Attributes: __VA_ARGS__ */

// ============================================================================
// O3DE_STRUCT - Mark a standalone struct for reflection (like USTRUCT)
// ============================================================================
//
// Attributes:
//   BlueprintType - Expose to Lua/ScriptCanvas
//   Atomic        - Always serialized as a whole unit
//   Category      - Category for organization
//   Description   - Tooltip description
//
// Example:
//   O3DE_STRUCT(BlueprintType, Category="Data")
//   struct DamageInfo
//   {
//       O3DE_PROPERTY(EditAnywhere)
//       float damage = 0.0f;
//       
//       O3DE_PROPERTY(EditAnywhere)
//       AZ::EntityId instigator;
//   };
//
#define O3DE_STRUCT(...) \
    /* This macro is parsed by O3DEReflect code generator */ \
    /* Attributes: __VA_ARGS__ */

// ============================================================================
// O3DE_ENUM - Mark an enum for reflection (like UENUM)
// ============================================================================
//
// Attributes:
//   BlueprintType - Expose to Lua/ScriptCanvas
//   Flags         - Treat as bitmask/flags enum
//   Category      - Category for organization
//   Description   - Tooltip description
//
// Use O3DE_ENUM_VALUE to provide display names for values:
//
// Example:
//   O3DE_ENUM(BlueprintType, Category="AI")
//   enum class AIState
//   {
//       O3DE_ENUM_VALUE(Idle, DisplayName="Standing Idle")
//       Idle,
//       
//       O3DE_ENUM_VALUE(Patrol, DisplayName="Patrolling")
//       Patrol,
//       
//       O3DE_ENUM_VALUE(Combat, DisplayName="In Combat")
//       Combat
//   };
//
#define O3DE_ENUM(...) \
    /* This macro is parsed by O3DEReflect code generator */ \
    /* Attributes: __VA_ARGS__ */

#define O3DE_ENUM_VALUE(Value, ...) \
    /* Attributes for enum value: __VA_ARGS__ */

// ============================================================================
// O3DE_GENERATED_BODY - Include generated code in class body
// ============================================================================
//
// Place this at the beginning of your class body to include generated code:
//
// Example:
//   O3DE_CLASS(Category="Gameplay")
//   class MyComponent : public AZ::Component
//   {
//       O3DE_GENERATED_BODY()
//   public:
//       // Your code here
//   };
//
// This will be replaced with #include "MyComponent.generated.h" pointing to
// the auto-generated reflection code.
//
#define O3DE_GENERATED_BODY() \
    /* Placeholder for generated includes - replaced during code generation */

// ============================================================================
// O3DE_IMPLEMENT_REFLECT - Generate Reflect() implementation
// ============================================================================
//
// Place in .cpp file to include generated Reflect() implementation:
//
// Example (in MyComponent.cpp):
//   O3DE_IMPLEMENT_REFLECT(MyComponent)
//
// This generates the full Reflect() function with SerializeContext,
// EditContext, and BehaviorContext registrations based on O3DE_PROPERTY
// and O3DE_FUNCTION annotations.
//
#define O3DE_IMPLEMENT_REFLECT(ClassName) \
    /* Placeholder - include generated source file */ \
    /* #include "ClassName.generated.cpp" */
