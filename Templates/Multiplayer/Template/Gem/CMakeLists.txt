
# Setup the gem meta variables that includes the "gem_path", "gem_name" and "gem_version"
# The "pal_dir" variable is also set which refers to the Platform/<platform-name> directory
# based on the directory of this CMakeLists.txt
o3de_gem_setup("${Name}")

# Now that we have the platform abstraction layer (PAL) folder for this folder, thats where we will find the
# traits for this platform. Traits for a platform are defines for things like whether or not something in this project
# is supported by this platform.
include(${pal_dir}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)

# Now that we have loaded our project traits for this platform, see if this project is even supported on this platform.
# If its not supported we just return after including the unsupported.
if(NOT PAL_TRAIT_${NameUpper}_SUPPORTED)
    return()
endif()

# We are on a supported platform, so add the ${gem_name} target
# Note: We include the common files and the platform specific files which are set in ${NameLower}_files.cmake and
# in ${pal_dir}/${NameLower}_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake

ly_add_target(
    NAME ${gem_name}.Client.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_autogen_files.cmake
        ${NameLower}_files.cmake
        ${NameLower}_client_files.cmake
        ${pal_dir}/${NameLower}_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::DebugDraw
            Gem::LyShine
            Gem::StartingPointInput
            Gem::EMotionFXStaticLib
            Gem::PhysX
            Gem::Multiplayer.Client
            Gem::AudioSystem.API
        PRIVATE
            Gem::LmbrCentral.Static
            Gem::Multiplayer.Client.Static
            Gem::PhysX.Static
            Gem::DebugDraw.Static
            Gem::ImGui.Static
            Gem::LyShine.Static
            Gem::GameState.Static
    AUTOGEN_RULES
        *.AutoComponent.xml,AutoComponent_Header.jinja,$path/$fileprefix.AutoComponent.h
        *.AutoComponent.xml,AutoComponent_Source.jinja,$path/$fileprefix.AutoComponent.cpp
        *.AutoComponent.xml,AutoComponentTypes_Header.jinja,$path/AutoComponentTypes.h
        *.AutoComponent.xml,AutoComponentTypes_Source.jinja,$path/AutoComponentTypes.cpp
)

ly_add_target(
    NAME ${gem_name}.Server.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_autogen_files.cmake
        ${NameLower}_files.cmake
        ${NameLower}_server_files.cmake
        ${pal_dir}/${NameLower}_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::StartingPointInput
            Gem::EMotionFXStaticLib
            Gem::PhysX
            Gem::Multiplayer.Server
        PRIVATE
            Gem::LmbrCentral.Static
            Gem::Multiplayer.Server.Static
            Gem::PhysX.Static
            Gem::GameState.Static
    AUTOGEN_RULES
        *.AutoComponent.xml,AutoComponent_Header.jinja,$path/$fileprefix.AutoComponent.h
        *.AutoComponent.xml,AutoComponent_Source.jinja,$path/$fileprefix.AutoComponent.cpp
        *.AutoComponent.xml,AutoComponentTypes_Header.jinja,$path/AutoComponentTypes.h
        *.AutoComponent.xml,AutoComponentTypes_Source.jinja,$path/AutoComponentTypes.cpp
)

ly_add_target(
    NAME ${gem_name}.Unified.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_autogen_files.cmake
        ${NameLower}_files.cmake
        ${NameLower}_client_files.cmake
        ${NameLower}_server_files.cmake
        ${pal_dir}/${NameLower}_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::DebugDraw
            Gem::LyShine
            Gem::StartingPointInput
            Gem::EMotionFXStaticLib
            Gem::PhysX
            Gem::Multiplayer
            Gem::AudioSystem.API
        PRIVATE
            Gem::LmbrCentral.Static
            Gem::Multiplayer.Unified.Static
            Gem::PhysX.Static
            Gem::DebugDraw.Static
            Gem::ImGui.Static
            Gem::LyShine.Static
            Gem::GameState.Static
    AUTOGEN_RULES
        *.AutoComponent.xml,AutoComponent_Header.jinja,$path/$fileprefix.AutoComponent.h
        *.AutoComponent.xml,AutoComponent_Source.jinja,$path/$fileprefix.AutoComponent.cpp
        *.AutoComponent.xml,AutoComponentTypes_Header.jinja,$path/AutoComponentTypes.h
        *.AutoComponent.xml,AutoComponentTypes_Source.jinja,$path/AutoComponentTypes.cpp
)

ly_add_target(
    NAME ${gem_name}.Client ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_shared_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}/Azcg/Generated/${gem_name}.Client.Static  # Used so Module.cpp can include AutoGen/AutoComponentTypes.h
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::Multiplayer.Client.Static
            Gem::${gem_name}.Client.Static
            Gem::Atom_AtomBridge.Static
)

ly_add_target(
    NAME ${gem_name}.Server ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_shared_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}/Azcg/Generated/${gem_name}.Server.Static  # Used so Module.cpp can include AutoGen/AutoComponentTypes.h
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::Multiplayer.Server.Static
            Gem::${gem_name}.Server.Static
            Gem::Atom_AtomBridge.Static
)

ly_add_target(
    NAME ${gem_name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ${NameLower}_shared_files.cmake
        ${pal_dir}/${NameLower}_shared_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
    INCLUDE_DIRECTORIES
        PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}/Azcg/Generated/${gem_name}.Unified.Static  # Used so Module.cpp can include AutoGen/AutoComponentTypes.h
            Source
            .
        PUBLIC
            Include
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::Multiplayer.Unified.Static
            Gem::${gem_name}.Unified.Static
            AZ::AzCore
)

# Include the gem name into the Client Module source file
# for use with the AZ_DECLARE_MODULE_CLASS macro
# This is to allow renaming of the gem to also cause
# the CreateModuleClass_Gem_<gem-name> function which
# is used to bootstrap the gem in monolithic builds to link to the new gem name
ly_add_source_properties(
SOURCES
    Source/${Name}Module.cpp
PROPERTY COMPILE_DEFINITIONS
    VALUES
        O3DE_GEM_NAME=${gem_name}
        O3DE_GEM_VERSION=${gem_version})

# if enabled, ${gem_name} is used by all kinds of applications
ly_create_alias(NAME ${gem_name}.Builders NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Tools    NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Clients  NAMESPACE Gem TARGETS Gem::${gem_name}.Client)
ly_create_alias(NAME ${gem_name}.Servers  NAMESPACE Gem TARGETS Gem::${gem_name}.Server)
ly_create_alias(NAME ${gem_name}.Unified  NAMESPACE Gem TARGETS Gem::${gem_name})

# Add in CMake dependencies for each gem dependency listed in this gem's gem.json file
# for the primary gem variants used in most o3de applications
o3de_add_variant_dependencies_for_gem_dependencies(GEM_NAME ${gem_name} VARIANTS Builders Tools Clients Servers Unified)

################################################################################
# Gem dependencies
################################################################################

# Query the project name from the nearest project.json file in a directory at or above
# the current one.
# This gem is the project gem and therefore should be part of the project that is using it
o3de_find_ancestor_project_root(project_path project_name "${CMAKE_CURRENT_SOURCE_DIR}")

# If the project name could not be queried from a project.json file, then fallback
# to using the name of the project provided when the project template was instantiated
if (NOT project_name)
    set(project_name ${Name})
endif()

# Add project to the list server projects to create the ${gem_name}.ServerLauncher
if(PAL_TRAIT_BUILD_SERVER_SUPPORTED)
    set_property(GLOBAL APPEND PROPERTY LY_LAUNCHER_SERVER_PROJECTS ${Name})
endif()

# Add project to the unified projects to create the ${gem_name}.UnifiedLauncher
if(PAL_TRAIT_BUILD_UNIFIED_SUPPORTED)
    set_property(GLOBAL APPEND PROPERTY LY_LAUNCHER_UNIFIED_PROJECTS ${Name})
endif()

set_property(TARGET ${Name} APPEND PROPERTY GAMELAUNCHER_ADDITIONAL_VS_DEBUGGER_COMMAND_ARGUMENTS "--console-command-file=\"launch_client.cfg\"")
set_property(TARGET ${Name} APPEND PROPERTY SERVERLAUNCHER_ADDITIONAL_VS_DEBUGGER_COMMAND_ARGUMENTS "--console-command-file=\"launch_server.cfg\"")
set_property(TARGET ${Name} APPEND PROPERTY UNIFIEDLAUNCHER_ADDITIONAL_VS_DEBUGGER_COMMAND_ARGUMENTS "--console-command-file=\"launch_server.cfg\"")