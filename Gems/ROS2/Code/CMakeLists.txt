# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT


# Only allow this gem to be configured on platforms that are currently supported
include(${CMAKE_CURRENT_SOURCE_DIR}/Platform/${PAL_PLATFORM_NAME}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)
if(NOT PAL_TRAIT_BUILD_ROS2_GEM_SUPPORTED)
    message(FATAL_ERROR "The ROS2 Gem is not currently supported on ${PAL_PLATFORM_NAME}")
    return()
endif()

# If no ROS2 is found, no targets are valid for this Gem
find_package(ROS2 MODULE)
if (NOT ROS2_FOUND)
    message(FATAL_ERROR "Unable to detect a the ROS2 distribution on this system. Make sure it is installed and enabled.")
    return()
endif()

# Add the ROS2.Static target
# Note: We include the common files and the platform specific files which are set in ros2_common_files.cmake
# and in ${pal_dir}/ros2_${PAL_PLATFORM_NAME_LOWERCASE}_files.cmake
ly_add_target(
    NAME ${gem_name}.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        ros2_header_files.cmake
        ros2_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            Gem::Atom_RPI.Public
            Gem::Atom_Feature_Common.Static
            Gem::Atom_Component_DebugCamera.Static
            Gem::StartingPointInput
)

target_depends_on_ros2_packages(${gem_name}.Static rclcpp builtin_interfaces std_msgs sensor_msgs nav_msgs urdfdom tf2_ros ackermann_msgs gazebo_msgs control_toolbox)

ly_add_target(
    NAME ${gem_name}.API HEADERONLY
    NAMESPACE Gem
    FILES_CMAKE
        ros2_header_files.cmake
    INCLUDE_DIRECTORIES
        INTERFACE
            Include
)

# Here add ROS2 target, it depends on the ROS2.Static
ly_add_target(
    NAME ${gem_name} ${PAL_TRAIT_MONOLITHIC_DRIVEN_MODULE_TYPE}
    NAMESPACE Gem
    FILES_CMAKE
        ros2_shared_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PRIVATE
            Gem::${gem_name}.Static
            Gem::Atom_Feature_Common.Static
)

# By default, we will specify that the above target ROS2 would be used by
# Client and Server type targets when this gem is enabled.  If you don't want it
# active in Clients or Servers by default, delete one of both of the following lines:
ly_create_alias(NAME ${gem_name}.Clients NAMESPACE Gem TARGETS Gem::${gem_name})
ly_create_alias(NAME ${gem_name}.Servers NAMESPACE Gem TARGETS Gem::${gem_name})

# If we are on a host platform, we want to add the host tools targets like the ${gem_name}.Editor target which
# will also depend on ${gem_name}.Static
if(PAL_TRAIT_BUILD_HOST_TOOLS)
    ly_add_target(
        NAME ${gem_name}.Editor.Static STATIC
        NAMESPACE Gem
        FILES_CMAKE
            ros2_editor_files.cmake
        PLATFORM_INCLUDE_FILES
            ${CMAKE_CURRENT_LIST_DIR}/Platform/Common/${PAL_TRAIT_COMPILER_ID}/ros2_static_editor_${PAL_TRAIT_COMPILER_ID_LOWERCASE}.cmake
        INCLUDE_DIRECTORIES
            PRIVATE
                Source
            PUBLIC
                Include
        COMPILE_DEFINITIONS
            PRIVATE
                ROS2_EDITOR
        BUILD_DEPENDENCIES
            PUBLIC
                AZ::AzToolsFramework
                Gem::AtomLyIntegration_CommonFeatures.Editor.Static
                Gem::LmbrCentral.API
                Gem::PhysX.Editor.Static
                Gem::${gem_name}.Static
    )

    ly_add_target(
        NAME ${gem_name}.Editor GEM_MODULE
        NAMESPACE Gem
        AUTOMOC
        AUTORCC
        FILES_CMAKE
            ros2_editor_files.cmake
            ros2_editor_shared_files.cmake
        PLATFORM_INCLUDE_FILES
            ${CMAKE_CURRENT_LIST_DIR}/Platform/Common/${PAL_TRAIT_COMPILER_ID}/ros2_static_editor_${PAL_TRAIT_COMPILER_ID_LOWERCASE}.cmake
        COMPILE_DEFINITIONS
            PRIVATE
                ROS2_EDITOR
        INCLUDE_DIRECTORIES
            PRIVATE
                Source
            PUBLIC
                Include
        BUILD_DEPENDENCIES
            PUBLIC
                Gem::${gem_name}.Editor.Static
                Gem::Atom_Feature_Common.Static
    )

    # By default, we will specify that the above target ROS2 would be used by
    # Tool and Builder type targets when this gem is enabled.  If you don't want it
    # active in Tools or Builders by default, delete one of both of the following lines:
    ly_create_alias(NAME ${gem_name}.Tools    NAMESPACE Gem TARGETS Gem::${gem_name}.Editor)
    ly_create_alias(NAME ${gem_name}.Builders NAMESPACE Gem TARGETS Gem::${gem_name}.Editor)
endif()

################################################################################
# Tests
################################################################################
# See if globally, tests are supported
if(PAL_TRAIT_BUILD_TESTS_SUPPORTED)
    if(PAL_TRAIT_TEST_GOOGLE_TEST_SUPPORTED)
        # We support ROS2.Tests on this platform, add ROS2.Tests target which depends on ROS2.Static
        ly_add_target(
            NAME ${gem_name}.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
            NAMESPACE Gem
            FILES_CMAKE
                ros2_tests_files.cmake
            INCLUDE_DIRECTORIES
                PRIVATE
                    Tests
                    Source
            BUILD_DEPENDENCIES
                PRIVATE
                    AZ::AzTest
                    Gem::${gem_name}.Static
        )

        # Add ROS2.Tests to googletest
        ly_add_googletest(
            NAME Gem::${gem_name}.Tests
        )
    endif()

    # If we are a host platform we want to add tools test like editor tests here
    if(PAL_TRAIT_BUILD_HOST_TOOLS)
        if(PAL_TRAIT_TEST_GOOGLE_TEST_SUPPORTED)
            # We support ROS2.Editor.Tests on this platform, add ROS2.Editor.Tests target which depends on ROS2.Editor
            ly_add_target(
                NAME ${gem_name}.Editor.Tests ${PAL_TRAIT_TEST_TARGET_TYPE}
                NAMESPACE Gem
                FILES_CMAKE
                    ros2_editor_tests_files.cmake
                PLATFORM_INCLUDE_FILES
                    ${CMAKE_CURRENT_LIST_DIR}/Platform/Common/${PAL_TRAIT_COMPILER_ID}/ros2_static_editor_${PAL_TRAIT_COMPILER_ID_LOWERCASE}.cmake
                INCLUDE_DIRECTORIES
                    PRIVATE
                        Tests
                        Source
                BUILD_DEPENDENCIES
                    PRIVATE
                        AZ::AzTest
                        Gem::${gem_name}.Editor
            )

            # Add ROS2.Editor.Tests to googletest
            ly_add_googletest(
                NAME Gem::${gem_name}.Editor.Tests
            )
        endif()
    endif()
endif()
