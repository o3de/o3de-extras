# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

# Currently we are in the Code folder: ${CMAKE_CURRENT_LIST_DIR}
# Get the platform specific folder ${pal_dir} for the current folder: ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME}
# Note: o3de_pal_dir will take care of the details for us, as this may be a restricted platform
#       in which case it will see if that platform is present here or in the restricted folder.
#       i.e. It could here in our gem : Gems/ROS2/Code/Platform/<platorm_name>  or
#            <restricted_folder>/<platform_name>/Gems/ROS2/Code
o3de_pal_dir(pal_dir ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

# Now that we have the platform abstraction layer (PAL) folder for this folder, thats where we will find the
# traits for this platform. Traits for a platform are defines for things like whether or not something in this gem
# is supported by this platform.
include(${pal_dir}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)

# Check to see if building the Gem Modules are supported for the current platform
if(NOT PAL_TRAIT_ROS2_SUPPORTED)
    message(FATAL_ERROR "The ROS2 Gem is not supported on ${PAL_PLATFORM_NAME}")
    return()
endif()

# If no ROS2 is found, no targets are valid for this Gem
find_package(ROS2 MODULE)
if (NOT ROS2_FOUND)
    message(FATAL_ERROR "Unable to detect a the ROS2 distribution on this system. Make sure it is installed and enabled.")
    return()
endif()
message(DEBUG "Building ${gem_name} Gem with ros2 $ENV{ROS_DISTRO}")

# Check if ROS 2 distribution is cached
get_property(ROS_DISTRO_TYPE CACHE ROS_DISTRO PROPERTY TYPE)

# Perform checks with cached ROS 2 distribution and sourced distribution. Save the distribution into cache if it was not cached before or if they do not match end with error.
if (NOT ROS_DISTRO_TYPE)
    set(ROS_DISTRO $ENV{ROS_DISTRO} CACHE STRING "ROS 2 distribution of current configuration" FORCE)
elseif(NOT "$ENV{ROS_DISTRO}" STREQUAL "${ROS_DISTRO}")
    get_cmake_property(CACHED_VARIABLES CACHE_VARIABLES)
    set(ROS_DISTRO_COPY ${ROS_DISTRO})
    # Iterate over cached variables and unset them
    foreach(CACHED_VARIABLE ${CACHED_VARIABLES})
        unset(${CACHED_VARIABLE} CACHE)
    endforeach()
    message(FATAL_ERROR "ROS 2 distribution does not match. Configuration created for: ${ROS_DISTRO_COPY}, sourced distribution: $ENV{ROS_DISTRO}.  Removed invalid configuration, please reconfigure project.")
endif()

# Add a custom target to always check during build if sourced and cached distributions match.
add_custom_target(
    ROS2DistributionCheck ALL
    COMMENT "Checking if sourced ROS 2 distribution matches with the configuration"
    COMMAND ${CMAKE_COMMAND} -DROS_DISTRO=${ROS_DISTRO} -P ${CMAKE_CURRENT_SOURCE_DIR}/checkROS2Distribution.cmake
)

# Add the static target with the API Interface and the code
# TODO: This target should be removed after API is fully implemented with buses
ly_add_target(
    NAME ${gem_name}.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ros2_api_files.cmake
        ros2_shared_api_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzToolsFramework
            Gem::${gem_name}.Private.Object
)
ly_add_target(
    NAME ${gem_name}.Editor.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ros2_editor_api_files.cmake
        ros2_editor_shared_api_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzToolsFramework
            Gem::${gem_name}.Editor.Private.Object
)


# The ${gem_name}.API target declares the common interface that users of this gem should depend on in their targets
ly_add_target(
    NAME ${gem_name}.API INTERFACE
    NAMESPACE Gem
    FILES_CMAKE
        ros2_api_files.cmake
        ${pal_dir}/ros2_api_files.cmake
    INCLUDE_DIRECTORIES
        INTERFACE
            Include
    BUILD_DEPENDENCIES
        INTERFACE
            AZ::AzCore
)

target_interface_depends_on_ros2_packages(${gem_name}.API rclcpp builtin_interfaces control_msgs geometry_msgs)

# The ${gem_name}.Private.Object target is an internal target
# It should not be used outside of this Gems CMakeLists.txt
ly_add_target(
    NAME ${gem_name}.Private.Object STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ros2_private_files.cmake
        ${pal_dir}/ros2_private_files.cmake
    TARGET_PROPERTIES
        O3DE_PRIVATE_TARGET TRUE
    INCLUDE_DIRECTORIES
        PRIVATE
            Include
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            Gem::Atom_AtomBridge.Static # FollowingCameraComponent
            Gem::LevelGeoreferencing.API # ROS2SpawnerComponent
)

target_depends_on_ros2_packages(${gem_name}.Private.Object rclcpp builtin_interfaces std_msgs sensor_msgs nav_msgs tf2_ros ackermann_msgs gazebo_msgs vision_msgs control_msgs)

# Here add ${gem_name} target, it depends on the Private Object library and Public API interface
ly_add_target(
    NAME ${gem_name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ros2_shared_files.cmake
        ${pal_dir}/ros2_shared_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            Gem::${gem_name}.API
        PRIVATE
            Gem::${gem_name}.Static
            Gem::${gem_name}.Private.Object
)

# Include the gem name into the Client Module source file
# for use with the AZ_DECLARE_MODULE_CLASS macro
# This is to allow renaming of the gem to also cause
# the CreateModuleClass_Gem_<gem-name> function which
# is used to bootstrap the gem in monolithic builds to link to the new gem name
ly_add_source_properties(
SOURCES
    Source/Clients/ROS2Module.cpp
PROPERTY COMPILE_DEFINITIONS
    VALUES
        O3DE_GEM_NAME=${gem_name}
        O3DE_GEM_VERSION=${gem_version})

# By default, we will specify that the above target ${gem_name} would be used by
# Client and Server type targets when this gem is enabled.  If you don't want it
# active in Clients or Servers by default, delete one of both of the following lines:
ly_create_alias(NAME ${gem_name}.Clients NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Servers NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Unified NAMESPACE Gem TARGETS Gem::${gem_name})

# For the Client and Server variants of ${gem_name} Gem, an alias to the ${gem_name}.API target will be made
ly_create_alias(NAME ${gem_name}.Clients.API NAMESPACE Gem TARGETS Gem::${gem_name}.API)
ly_create_alias(NAME ${gem_name}.Servers.API NAMESPACE Gem TARGETS Gem::${gem_name}.API)
ly_create_alias(NAME ${gem_name}.Unified.API NAMESPACE Gem TARGETS Gem::${gem_name}.API)

# Add in CMake dependencies for each gem dependency listed in this gem's gem.json file
# for the Clients, Servers, Unified gem variants
o3de_add_variant_dependencies_for_gem_dependencies(GEM_NAME ${gem_name} VARIANTS Clients Servers Unified)

# If we are on a host platform, we want to add the host tools targets like the ${gem_name}.Editor MODULE target
if(PAL_TRAIT_BUILD_HOST_TOOLS)
    # The ${gem_name}.Editor.API target can be used by other gems that want to interact with the ${gem_name}.Editor module
    ly_add_target(
        NAME ${gem_name}.Editor.API INTERFACE
        NAMESPACE Gem
        FILES_CMAKE
            ros2_editor_api_files.cmake
            ${pal_dir}/ros2_editor_api_files.cmake
        INCLUDE_DIRECTORIES
            INTERFACE
                Include
        BUILD_DEPENDENCIES
            INTERFACE
                AZ::AzToolsFramework
    )

    # The ${gem_name}.Editor.Private.Object target is an internal target
    # which is only to be used by this gems CMakeLists.txt and any subdirectories
    # Other gems should not use this target
    ly_add_target(
        NAME ${gem_name}.Editor.Private.Object STATIC
        NAMESPACE Gem
        FILES_CMAKE
            ros2_editor_private_files.cmake
        TARGET_PROPERTIES
            O3DE_PRIVATE_TARGET TRUE
        INCLUDE_DIRECTORIES
            PRIVATE
                Include
                Source
        COMPILE_DEFINITIONS
            PRIVATE
                ROS2_EDITOR
        BUILD_DEPENDENCIES
            PUBLIC
                AZ::AzToolsFramework
                Gem::${gem_name}.Static
    )

    ly_add_target(
        NAME ${gem_name}.Editor GEM_MODULE
        NAMESPACE Gem
        AUTOMOC
        FILES_CMAKE
            ros2_editor_shared_files.cmake
        INCLUDE_DIRECTORIES
            PRIVATE
                Source
            PUBLIC
                Include
        BUILD_DEPENDENCIES
            PUBLIC
                Gem::${gem_name}.Editor.API
            PRIVATE
                Gem::${gem_name}.Editor.Private.Object
                Gem::${gem_name}.Editor.Static
    )

    # Include the gem name into the Editor Module source file
    # for use with the AZ_DECLARE_MODULE_CLASS macro
    # This is to allow renaming of the gem to also cause
    # the CreateModuleClass_Gem_<gem-name> function which
    # is used to bootstrap the gem in monolithic builds to link to the new gem name
    ly_add_source_properties(
    SOURCES
        Source/Tools/ROS2EditorModule.cpp
    PROPERTY COMPILE_DEFINITIONS
        VALUES
            O3DE_GEM_NAME=${gem_name}
            O3DE_GEM_VERSION=${gem_version})

    # By default, we will specify that the above target ${gem_name} would be used by
    # Tool and Builder type targets when this gem is enabled.  If you don't want it
    # active in Tools or Builders by default, delete one of both of the following lines:
    ly_create_alias(NAME ${gem_name}.Tools    NAMESPACE Gem TARGETS Gem::${gem_name}.Editor)
    ly_create_alias(NAME ${gem_name}.Builders NAMESPACE Gem TARGETS Gem::${gem_name}.Editor)

    # For the Tools and Builders variants of ${gem_name} Gem, an alias to the ${gem_name}.Editor API target will be made
    ly_create_alias(NAME ${gem_name}.Tools.API NAMESPACE Gem TARGETS Gem::${gem_name}.Editor.API)
    ly_create_alias(NAME ${gem_name}.Builders.API NAMESPACE Gem TARGETS Gem::${gem_name}.Editor.API)

    # Add in CMake dependencies for each gem dependency listed in this gem's gem.json file
    # for the Tools and Builders gem variants
    o3de_add_variant_dependencies_for_gem_dependencies(GEM_NAME ${gem_name} VARIANTS Tools Builders)
endif()

################################################################################
# Tests
################################################################################
# See if globally, tests are supported
if(PAL_TRAIT_BUILD_TESTS_SUPPORTED)
    # We globally support tests, see if we support tests on this platform for ${gem_name}.Tests
    if(PAL_TRAIT_ROS2_TEST_SUPPORTED)
        # We support ${gem_name}.Tests on this platform, add dependency on the Private Object target
        ly_add_target(
            NAME ${gem_name}.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
            NAMESPACE Gem
            FILES_CMAKE
                ros2_tests_files.cmake
            INCLUDE_DIRECTORIES
                PRIVATE
                    Tests
                    Source
                    Include
            BUILD_DEPENDENCIES
                PRIVATE
                    AZ::AzTest
                    AZ::AzFramework
                    Gem::${gem_name}.Private.Object
        )

        # Add ${gem_name}.Tests to googletest
        ly_add_googletest(
            NAME Gem::${gem_name}.Tests
        )
    endif()

    # If we are a host platform we want to add tools test like editor tests here
    if(PAL_TRAIT_BUILD_HOST_TOOLS)
        # We are a host platform, see if Editor tests are supported on this platform
        if(PAL_TRAIT_ROS2_EDITOR_TEST_SUPPORTED)
            # We support ${gem_name}.Editor.Tests on this platform, add ${gem_name}.Editor.Tests target which depends on
            # private ${gem_name}.Editor.Private.Object target
            ly_add_target(
                NAME ${gem_name}.Editor.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
                NAMESPACE Gem
                FILES_CMAKE
                    ros2_editor_tests_files.cmake
                INCLUDE_DIRECTORIES
                    PRIVATE
                        Tests
                        Source
                        Include
                BUILD_DEPENDENCIES
                    PRIVATE
                        AZ::AzTest
                        AZ::AzTestShared
                        Legacy::CryCommon
                        Legacy::EditorCommon
                        Legacy::Editor.Headers
                        AZ::AzManipulatorTestFramework.Static
                        Gem::${gem_name}.Editor.Private.Object
                        Gem::${gem_name}.Editor.Static
                        Gem::${gem_name}.Static
                RUNTIME_DEPENDENCIES
                    Legacy::Editor
            )

            # Add ${gem_name}.Editor.Tests to googletest
            ly_add_googletest(
                NAME Gem::${gem_name}.Editor.Tests
            )
        endif()
    endif()
endif()
