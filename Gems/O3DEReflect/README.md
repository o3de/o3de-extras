# O3DE Reflect

O3DE Reflect is a auto c++ generation gem that dramatically simplifies component, struct, and enum creation in O3DE.

## Two Approaches

O3DE Reflect supports two ways to define your reflected types:

| Approach | Best For | Style |
|----------|----------|-------|
|  XML Definitions | New projects, clean separation | XML files (`.O3DEReflect.xml`) |
|  Header Macros | Existing code, Unreal-familiar devs | C++ macros in headers |

## Benefits

- Faster Iteration Time 
    The plugin allows you to create plugins faster, and saves time reflecting it to the editor, freeing up space in your cpp files.

- Multiplayer Binding Support
---

Header Macros (Unreal-Style)

The standard approach lets you write clean C++ with Unreal-style macros:

### Quick Example

```cpp
#include <O3DEReflect/O3DEReflectMacros.h>

O3DE_CLASS(Category = "Gameplay", Description = "Player movement controller")
class PlayerMovementComponent : public AZ::Component
{
    O3DE_GENERATED_BODY()
    O3DE_COMPONENT(PlayerMovementComponent, "{12345678-...}")

public:
    O3DE_PROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Movement", Tooltip = "Max speed in m/s",
        Min = 0.0f, Max = 100.0f, Suffix = "m/s")
    float m_maxSpeed = 10.0f;

    O3DE_PROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "State")
    bool m_isGrounded = false;

    O3DE_FUNCTION(BlueprintCallable, Category = "Movement")
    void Move(const AZ::Vector3& direction);

    O3DE_FUNCTION(BlueprintPure, Category = "State")
    float GetCurrentSpeed() const;
};
```

### Macros Reference

| Macro | Purpose | Like Unreal's |
|-------|---------|---------------|
| `O3DE_CLASS(...)` | Mark class for reflection | `UCLASS()` |
| `O3DE_COMPONENT(Name, UUID, ...)` | Declare component with services | `UCLASS()` |
| `O3DE_PROPERTY(...)` | Mark member for serialization | `UPROPERTY()` |
| `O3DE_FUNCTION(...)` | Expose function to scripts | `UFUNCTION()` |
| `O3DE_STRUCT(...)` | Mark struct for reflection | `USTRUCT()` |
| `O3DE_ENUM(...)` | Mark enum for reflection | `UENUM()` |
| `O3DE_ENUM_VALUE(...)` | Add enum value metadata | - |
| `O3DE_GENERATED_BODY()` | Include generated code | `GENERATED_BODY()` |

### Property Specifiers

**Visibility:**
- `EditAnywhere` - Editable on instances and prefabs
- `EditDefaultsOnly` - Editable only on prefabs/archetypes
- `EditInstanceOnly` - Editable only on instances
- `VisibleAnywhere` - Visible but not editable

**Scripting:**
- `BlueprintReadWrite` - Read+write from Lua/ScriptCanvas
- `BlueprintReadOnly` - Read-only from Lua/ScriptCanvas

**Metadata:**
- `Category = "Group|SubGroup"` - Property grouping
- `Tooltip = "Description"` - Hover tooltip
- `DisplayName = "Custom Name"` - Override display name
- `Min = 0.0f, Max = 100.0f` - Value constraints
- `UIMin, UIMax` - Slider range (can differ from Min/Max)
- `Suffix = "m/s"` - Unit suffix
- `ChangeNotify = "OnValueChanged"` - Change callback

### Function Specifiers

- `BlueprintCallable` - Can be called from script canvas
- `BlueprintPure` - Pure function (can be cached, shown differently in graph)
- `CallInEditor` - Can be invoked from editor UI
- `Server` / `Client` / `NetMulticast` - Network replication (future)

###  CMake Integration

```cmake
include(O3DEReflect)

ly_add_target(NAME MyTarget ...)

# Explicit list of headers to parse
ly_enable_o3de_reflect_v2(
    TARGET MyTarget
    HEADERS
        Source/PlayerMovementComponent.h
        Source/AIStateComponent.h
        Source/DamageInfo.h
)
```



## XML Definitions

### 1. Define your component in XML

Create a file named `MyComponent.O3DEReflect.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<Component
    Name="MyComponent"
    Namespace="MyGame"
    Category="Gameplay"
    Description="My awesome gameplay component">

    <Service Name="MyComponentService" Type="Provides"/>
    <Service Name="TransformService" Type="Requires"/>

    <Property Name="speed" Type="float" Default="10.0f"
        Category="Movement"
        Tooltip="Movement speed in m/s"
        Min="0.0" Max="100.0"
        Suffix="m/s"
        EditAnywhere="true"
        BlueprintReadWrite="true"/>

    <Property Name="health" Type="int" Default="100"
        Category="Combat"
        Tooltip="Current health points"
        Min="0" Max="1000"
        EditAnywhere="true"
        BlueprintReadWrite="true"/>

    <Function Name="TakeDamage"
        Category="Combat"
        Tooltip="Apply damage to this component"
        BlueprintCallable="true">
        <Param Name="amount" Type="float"/>
        <Return Type="bool"/>
    </Function>

</Component>
```

### 2. Enable code generation in CMake

```cmake
include(O3DEReflect)

ly_add_target(
    NAME MyGem
    ...
)

ly_enable_o3de_reflect(TARGET MyGem)
```

### 3. Build your project

The generator creates:
- `MyComponent.AutoReflect.h` - Complete header with class definition
- `MyComponent.AutoReflect.cpp` - Complete implementation with Reflect() function

## XML Schema

### Component Definition

```xml
<Component
    Name="ComponentName"           <!-- Required: Class name -->
    Namespace="MyNamespace"        <!-- Optional: C++ namespace -->
    Uuid="{...}"                   <!-- Optional: Auto-generated if omitted -->
    Category="Category/SubCategory" <!-- Editor category -->
    Description="..."              <!-- Tooltip description -->
    Icon="Icons/MyIcon.svg"        <!-- Editor icon path -->
    HideInEditor="false"           <!-- Hide from Add Component menu -->
    AppearsInAddComponentMenu="Game"> <!-- Menu category (Game, UI, etc.) -->

    <!-- Includes -->
    <Include File="AzCore/Math/Vector3.h"/>

    <!-- Base classes -->
    <BaseClass Name="Component" Namespace="AZ" Include="AzCore/Component/Component.h"/>

    <!-- Services -->
    <Service Name="MyService" Type="Provides"/>    <!-- or Requires, Incompatible, Dependent -->

    <!-- Properties -->
    <Property Name="myProperty" Type="float" .../>

    <!-- Functions -->
    <Function Name="MyFunction" ...>
        <Param Name="param1" Type="int"/>
        <Return Type="bool"/>
    </Function>

</Component>
```

### Property Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `Name` | string | Property name (m_ prefix added automatically) |
| `Type` | string | C++ type |
| `Default` | string | Default value |
| `DisplayName` | string | Override display name in editor |
| `Category` | string | Property group in editor |
| `Tooltip` | string | Hover tooltip text |
| `Suffix` | string | Unit suffix (e.g., "m/s") |
| `EditAnywhere` | bool | Editable in editor (default: true) |
| `VisibleAnywhere` | bool | Visible but read-only in editor |
| `ReadOnly` | bool | Read-only property |
| `BlueprintReadWrite` | bool | Exposed to scripts read+write |
| `BlueprintReadOnly` | bool | Exposed to scripts read-only |
| `Min` / `Max` | float | Value constraints |
| `ChangeNotify` | string | Function to call on value change |

### Function Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `Name` | string | Function name |
| `DisplayName` | string | Override display name |
| `Category` | string | Script node category |
| `Tooltip` | string | Function description |
| `BlueprintCallable` | bool | Callable from scripts (default: true) |
| `BlueprintPure` | bool | Pure function (no side effects) |
| `CallInEditor` | bool | Can be called from editor buttons |

### Struct Definition (like USTRUCT)

```xml
<Struct
    Name="DamageInfo"
    Namespace="MyGame"
    Category="Combat"
    Description="Damage event data"
    BlueprintType="true">

    <Property Name="damage" Type="float" Default="0.0f" .../>
    <Property Name="source" Type="AZ::EntityId" .../>

</Struct>
```

### Enum Definition (like UENUM)

```xml
<Enum
    Name="AIState"
    Namespace="MyGame"
    Category="AI"
    Description="AI behavior states"
    BlueprintType="true"
    UnderlyingType="uint8_t">

    <EnumValue Name="Idle" Value="0" DisplayName="Standing Idle"/>
    <EnumValue Name="Patrol" Value="1" DisplayName="Patrolling"/>
    <EnumValue Name="Combat" Value="2" DisplayName="In Combat"/>

</Enum>
```

### Flags Enum (Bitfield)

```xml
<Enum Name="DamageFlags" Flags="true" UnderlyingType="uint32_t">
    <EnumValue Name="None" Value="0"/>
    <EnumValue Name="IgnoreArmor"/>        <!-- Auto: 1 << 0 -->
    <EnumValue Name="CanCrit"/>            <!-- Auto: 1 << 1 -->
    <EnumValue Name="ApplyKnockback"/>     <!-- Auto: 1 << 2 -->
</Enum>
```

## Generated Code

For a component with properties and functions, O3DEReflect generates:

### Header (.AutoReflect.h)
- Class declaration with `AZ_COMPONENT_DECL`
- Getters and setters for all properties
- Function declarations
- Member variables with defaults

### Source (.AutoReflect.cpp)
- `AZ_COMPONENT_IMPL` macro
- Complete `Reflect()` function with:
  - SerializeContext registration
  - EditContext configuration (categories, tooltips, constraints)
  - BehaviorContext bindings for scripting
- Service functions (`GetProvidedServices`, etc.)
- Empty `Init()`, `Activate()`, `Deactivate()` stubs
- Empty function implementations (to be filled in)

## Comparison: Traditional vs O3DEReflect

### Traditional O3DE (~200+ lines)
```cpp
// Header
class MyComponent : public AZ::Component {
    AZ_COMPONENT(MyComponent, "{UUID}");
    static void Reflect(AZ::ReflectContext* context);
    static void GetProvidedServices(...);
    // ... 50+ lines of declarations
};

// Source
void MyComponent::Reflect(AZ::ReflectContext* context) {
    // 100+ lines of SerializeContext, EditContext, BehaviorContext
}
void MyComponent::GetProvidedServices(...) { ... }
// ... more boilerplate
```

### O3DEReflect (~80 lines of XML)
```xml
<Component Name="MyComponent" Category="Gameplay" ...>
    <Service Name="MyService" Type="Provides"/>
    <Property Name="speed" Type="float" ... />
    <Function Name="DoSomething" ... />
</Component>
```

## Examples

See the `Examples/` directory for complete examples:


## CMake Integration

### XML Files

**Option A: ly_enable_o3de_reflect**
```cmake
include(O3DEReflect)

ly_add_target(
    NAME MyTarget
    ...
)

ly_enable_o3de_reflect(
    TARGET MyTarget
    INPUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Source
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/Generated
)
```

**Option B: AUTOGEN_RULES (AzAutoGen pattern)**
```cmake
ly_add_o3de_reflect_autogen_rules(O3DE_REFLECT_RULES)

ly_add_target(
    NAME MyTarget
    ...
    AUTOGEN_RULES ${O3DE_REFLECT_RULES}
)
```

###  Header Macros

```cmake
include(O3DEReflect)

ly_add_target(NAME MyTarget ...)

ly_enable_o3de_reflect_v2(
    TARGET MyTarget
    HEADERS
        Source/MyComponent.h
        Source/MyStruct.h
)
```

### Auto-Detection

```cmake
# Automatically enables V1 for any .O3DEReflect.xml files found
ly_enable_o3de_reflect_auto(TARGET MyTarget)
```

## Future Plans

- **IDE integration** - IntelliSense support for XML schema and macro completion
- **Custom templates** - User-defined generation patterns
- **Network replication** - Server/Client/NetMulticast specifiers for multiplayer
- **Slate-like UI** - UI property specifiers for custom widgets

---

## Troubleshooting

### "Jinja2 not available" Error

The generator needs access to O3DE's Python environment with Jinja2 installed. The system automatically searches for it in `~/.o3de/Python/venv/*/lib/site-packages`. If this fails:

1. Ensure O3DE's Python is set up by running `python/get_python.bat` from O3DE source
2. Verify `LY_PYTHON_CMD` is set in your CMake configuration
3. Check that Jinja2 is installed: `pip install jinja2`

### Duplicate Symbol Linker Warnings

If you have both a manual `.cpp` file and O3DEReflect generates one, you'll get LNK4006 warnings about duplicate symbols. Solutions:

1. **Recommended**: Remove your manual `.cpp` and let O3DEReflect generate everything
2. **Alternative**: Keep your `.cpp` but ensure it doesn't duplicate generated functions

### Bus Handlers in AZ_COMPONENT_IMPL

O3DEReflect automatically filters out EBus handlers (classes ending in `::Handler` or containing `BusHandler`) from:
- `AZ_COMPONENT_IMPL` macro arguments
- `SerializeContext::Class<>` template arguments

This is correct behavior - bus handlers aren't serializable base classes.

### CMake Cache Issues

If changes to O3DEReflect.cmake aren't being picked up:

```powershell
# Remove CMake cache and regenerate
Remove-Item build/windows/CMakeFiles -Recurse -Force
cmake -B build/windows -G "Visual Studio 17 2022" ...
```

### Generated Files Not Found

Check that the generated output directory is correct:
- Default: `${CMAKE_CURRENT_BINARY_DIR}/Generated/${TARGET_NAME}/`
- Files are named: `ComponentName.AutoReflect.h` and `ComponentName.AutoReflect.cpp`

### Multi-Target Projects

For gems with multiple targets (Client, Server, Unified), use the multi-target syntax:

```cmake
ly_enable_o3de_reflect_v2(
    TARGETS
        MyGem.Static
        MyGem.Client.Static
        MyGem.Server.Static
    HEADERS
        Source/MyComponent.h
)
```

---

## Architecture

```
O3DEReflect/
├── O3DEReflect.cmake                    # CMake module (main entry point)
├── README.md                            # This documentation
├── Code/
│   ├── CMakeLists.txt                   # Gem build configuration
│   └── Include/
│       └── O3DEReflect/
│           ├── O3DEReflectMacros.h      # C++ macro definitions
│           └── AutoGen/
│               ├── O3DEReflectGen.py           # Code generator
│               ├── O3DEReflectHeaderParser.py  # header parser
│               ├── O3DEReflect.xsd             #  XML schema
│               ├── O3DEReflect_Header.jinja    # Header template
│               ├── O3DEReflect_Source.jinja    # Source template
│               └── O3DEReflect_Common.jinja    # Shared Jinja macros
└── Examples/                            # Example files
```

### Build Flow

1. **CMake Configuration**: `ly_enable_o3de_reflect_v2()` registers custom commands
2. **Build Time**: For each header, runs `O3DEReflectHeaderParser.py`
3. **Parsing**: Extracts O3DE_* macros and metadata from C++ headers
4. **Generation**: `O3DEReflectGen.py` produces `.AutoReflect.h` and `.AutoReflect.cpp`
5. **Compilation**: Generated files are compiled with your target

---

## License

Apache-2.0 OR MIT.
