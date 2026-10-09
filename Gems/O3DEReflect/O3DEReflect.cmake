# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

#
# O3DEReflect CMake Module
#
# This module provides functions to enable O3DE Reflect code generation for targets.
# It integrates with the AzAutoGen system to generate reflection code from XML files.
#
# Usage:
#   include(O3DEReflect)
#   ly_enable_o3de_reflect(TARGET MyTarget)
#
# Or use AUTOGEN_RULES in ly_add_target:
#   ly_add_target(
#       NAME MyTarget
#       ...
#       AUTOGEN_RULES
#           *.O3DEReflect.xml,O3DEReflect_Header.jinja,$path/$fileprefix.AutoReflect.h
#           *.O3DEReflect.xml,O3DEReflect_Source.jinja,$path/$fileprefix.AutoReflect.cpp
#   )
#

# Find the O3DEReflect gem directory - use CMAKE_CURRENT_LIST_DIR since this file is in the gem
# Use FORCE to ensure the path is always recalculated when the file is re-included
# This fixes issues when deploying projects where cached paths may be stale
set(O3DE_REFLECT_GEM_DIR "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "Path to O3DEReflect gem" FORCE)

# Template directory
set(O3DE_REFLECT_TEMPLATE_DIR "${O3DE_REFLECT_GEM_DIR}/Code/Include/O3DEReflect/AutoGen" CACHE PATH "Path to O3DEReflect templates" FORCE)

# Python script for code generation (V1: XML)
set(O3DE_REFLECT_GENERATOR "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflectGen.py" CACHE FILEPATH "Path to O3DEReflect generator script" FORCE)

# Python script for header parsing (V2: Header macros)
set(O3DE_REFLECT_HEADER_PARSER "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflectHeaderParser.py" CACHE FILEPATH "Path to O3DEReflect header parser" FORCE)

# Use O3DE's Python which has Jinja2 installed
# LY_PYTHON_CMD is set by LYPython.cmake and points to O3DE's Python with all required packages
if(NOT DEFINED LY_PYTHON_CMD)
    # Fallback to Python_EXECUTABLE if LY_PYTHON_CMD is not set
    set(O3DE_REFLECT_PYTHON_CMD "${Python_EXECUTABLE}")
else()
    set(O3DE_REFLECT_PYTHON_CMD ${LY_PYTHON_CMD})
endif()

#! ly_enable_o3de_reflect: Enable O3DE Reflect code generation for a target
#
# This function sets up automatic code generation for O3DEReflect XML files.
# It will generate .AutoReflect.h and .AutoReflect.cpp files from *.O3DEReflect.xml
# input files in the target's source directories.
#
# \arg:TARGET - Name of the target to enable O3DE Reflect for
# \arg:INPUT_DIR - Optional input directory to search for XML files (defaults to target source dir)
# \arg:OUTPUT_DIR - Optional output directory for generated files (defaults to ${CMAKE_CURRENT_BINARY_DIR}/AutoGen)
#
function(ly_enable_o3de_reflect)
    set(options)
    set(oneValueArgs TARGET INPUT_DIR OUTPUT_DIR)
    set(multiValueArgs)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARG_TARGET)
        message(FATAL_ERROR "ly_enable_o3de_reflect requires a TARGET argument")
    endif()

    # Default output directory
    if(NOT ARG_OUTPUT_DIR)
        set(ARG_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/AutoGen/${ARG_TARGET}")
    endif()

    # Get source directory
    get_target_property(TARGET_SOURCE_DIR ${ARG_TARGET} SOURCE_DIR)
    if(NOT ARG_INPUT_DIR)
        set(ARG_INPUT_DIR "${TARGET_SOURCE_DIR}")
    endif()

    # Find all O3DEReflect XML files
    file(GLOB_RECURSE O3DE_REFLECT_XML_FILES 
        "${ARG_INPUT_DIR}/*.O3DEReflect.xml"
    )

    if(NOT O3DE_REFLECT_XML_FILES)
        message(STATUS "No O3DEReflect XML files found in ${ARG_INPUT_DIR}")
        return()
    endif()

    # Create output directory
    file(MAKE_DIRECTORY ${ARG_OUTPUT_DIR})

    # Generated file lists
    set(GENERATED_HEADERS)
    set(GENERATED_SOURCES)

    foreach(XML_FILE ${O3DE_REFLECT_XML_FILES})
        # Get file name without extension
        get_filename_component(FILE_NAME ${XML_FILE} NAME_WE)
        # Remove .O3DEReflect suffix if present
        string(REGEX REPLACE "\\.O3DEReflect$" "" FILE_PREFIX "${FILE_NAME}")

        # Output files - one per type defined in XML
        # For now, use the file prefix as the component name
        set(HEADER_FILE "${ARG_OUTPUT_DIR}/${FILE_PREFIX}.AutoReflect.h")
        set(SOURCE_FILE "${ARG_OUTPUT_DIR}/${FILE_PREFIX}.AutoReflect.cpp")

        # Add custom command to generate files
        add_custom_command(
            OUTPUT ${HEADER_FILE} ${SOURCE_FILE}
            COMMAND ${O3DE_REFLECT_PYTHON_CMD} "${O3DE_REFLECT_GENERATOR}"
                --input "${XML_FILE}"
                --output-dir "${ARG_OUTPUT_DIR}"
                --template-dir "${O3DE_REFLECT_TEMPLATE_DIR}"
            DEPENDS 
                ${XML_FILE}
                "${O3DE_REFLECT_GENERATOR}"
                "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Header.jinja"
                "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Source.jinja"
                "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Common.jinja"
            COMMENT "Generating O3DEReflect code from ${FILE_NAME}"
            VERBATIM
        )

        list(APPEND GENERATED_HEADERS ${HEADER_FILE})
        list(APPEND GENERATED_SOURCES ${SOURCE_FILE})
    endforeach()

    # Add generated files to target
    target_sources(${ARG_TARGET}
        PRIVATE
            ${GENERATED_HEADERS}
            ${GENERATED_SOURCES}
    )

    # Add include directory for generated headers
    target_include_directories(${ARG_TARGET}
        PRIVATE
            ${ARG_OUTPUT_DIR}
    )

    # Create a custom target for the generated files
    add_custom_target(${ARG_TARGET}_O3DEReflect_Generate
        DEPENDS ${GENERATED_HEADERS} ${GENERATED_SOURCES}
    )
    add_dependencies(${ARG_TARGET} ${ARG_TARGET}_O3DEReflect_Generate)

    message(STATUS "O3DEReflect enabled for ${ARG_TARGET}: ${O3DE_REFLECT_XML_FILES}")

endfunction()

#! ly_add_o3de_reflect_autogen_rules: Get AutoGen rules for O3DEReflect
#
# Returns the AUTOGEN_RULES suitable for ly_add_target.
# Use this if you prefer the AzAutoGen approach over ly_enable_o3de_reflect.
#
# Example:
#   ly_add_o3de_reflect_autogen_rules(RULES)
#   ly_add_target(
#       NAME MyTarget
#       AUTOGEN_RULES ${RULES}
#   )
#
function(ly_add_o3de_reflect_autogen_rules OUT_VAR)
    set(${OUT_VAR}
        "*.O3DEReflect.xml,${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Header.jinja,$path/$fileprefix.AutoReflect.h"
        "*.O3DEReflect.xml,${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Source.jinja,$path/$fileprefix.AutoReflect.cpp"
        PARENT_SCOPE
    )
endfunction()

#! ly_enable_o3de_reflect_v2: Enable V2 header-macro parsing for targets
#
# This function sets up automatic code generation from C++ header files
# that use O3DE_CLASS, O3DE_PROPERTY, O3DE_FUNCTION, etc. macros.
# It will generate .generated.h and .generated.cpp files.
#
# \arg:TARGET - One or more target names to enable O3DE Reflect V2 for
# \arg:HEADERS - List of header files to parse for O3DE macros
# \arg:OUTPUT_DIR - Optional output directory (defaults to ${CMAKE_CURRENT_BINARY_DIR}/Generated)
#
function(ly_enable_o3de_reflect_v2)
    set(options)
    set(oneValueArgs OUTPUT_DIR)
    set(multiValueArgs TARGET HEADERS)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARG_TARGET)
        message(FATAL_ERROR "ly_enable_o3de_reflect_v2 requires at least one TARGET argument")
    endif()

    if(NOT ARG_HEADERS)
        message(FATAL_ERROR "ly_enable_o3de_reflect_v2 requires HEADERS argument")
    endif()

    # Process each target
    foreach(CURRENT_TARGET ${ARG_TARGET})
        # Check if target exists
        if(NOT TARGET ${CURRENT_TARGET})
            message(WARNING "ly_enable_o3de_reflect_v2: Target '${CURRENT_TARGET}' does not exist, skipping")
            continue()
        endif()

        # Default output directory per target
        if(ARG_OUTPUT_DIR)
            set(TARGET_OUTPUT_DIR "${ARG_OUTPUT_DIR}/${CURRENT_TARGET}")
        else()
            set(TARGET_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/Generated/${CURRENT_TARGET}")
        endif()

        # Create output directory
        file(MAKE_DIRECTORY ${TARGET_OUTPUT_DIR})

        # Generated file lists
        set(GENERATED_HEADERS)
        set(GENERATED_SOURCES)

        foreach(HEADER_FILE ${ARG_HEADERS})
            # Get absolute path
            get_filename_component(HEADER_FILE_ABS "${HEADER_FILE}" ABSOLUTE)
            
            # Check if file exists
            if(NOT EXISTS "${HEADER_FILE_ABS}")
                message(WARNING "ly_enable_o3de_reflect_v2: Header file '${HEADER_FILE_ABS}' does not exist")
                continue()
            endif()
            
            # Get file name without extension
            get_filename_component(FILE_NAME ${HEADER_FILE} NAME_WE)

            # Output files (use .AutoReflect suffix to match Python generator output)
            set(GEN_HEADER "${TARGET_OUTPUT_DIR}/${FILE_NAME}.AutoReflect.h")
            set(GEN_SOURCE "${TARGET_OUTPUT_DIR}/${FILE_NAME}.AutoReflect.cpp")

            # Add custom command to generate files
            add_custom_command(
                OUTPUT ${GEN_HEADER} ${GEN_SOURCE}
                COMMAND ${O3DE_REFLECT_PYTHON_CMD} "${O3DE_REFLECT_HEADER_PARSER}"
                    --input "${HEADER_FILE_ABS}"
                    --output-dir "${TARGET_OUTPUT_DIR}"
                    --template-dir "${O3DE_REFLECT_TEMPLATE_DIR}"
                DEPENDS 
                    ${HEADER_FILE_ABS}
                    "${O3DE_REFLECT_HEADER_PARSER}"
                    "${O3DE_REFLECT_GENERATOR}"
                    "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Header.jinja"
                    "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Source.jinja"
                    "${O3DE_REFLECT_TEMPLATE_DIR}/O3DEReflect_Common.jinja"
                COMMENT "Generating O3DEReflect code from ${FILE_NAME}.h for ${CURRENT_TARGET}"
                VERBATIM
            )

            list(APPEND GENERATED_HEADERS ${GEN_HEADER})
            list(APPEND GENERATED_SOURCES ${GEN_SOURCE})
        endforeach()

        if(GENERATED_HEADERS)
            # Add generated files to target
            target_sources(${CURRENT_TARGET}
                PRIVATE
                    ${GENERATED_HEADERS}
                    ${GENERATED_SOURCES}
            )

            # Add include directory for generated headers
            target_include_directories(${CURRENT_TARGET}
                PRIVATE
                    ${TARGET_OUTPUT_DIR}
            )

            # Create a custom target for the generated files
            add_custom_target(${CURRENT_TARGET}_O3DEReflect_V2_Generate
                DEPENDS ${GENERATED_HEADERS} ${GENERATED_SOURCES}
            )
            add_dependencies(${CURRENT_TARGET} ${CURRENT_TARGET}_O3DEReflect_V2_Generate)

            message(STATUS "O3DEReflect V2 enabled for ${CURRENT_TARGET}: ${ARG_HEADERS}")
        endif()
    endforeach()
endfunction()

#! ly_enable_o3de_reflect_auto: Automatically detect and enable O3DE Reflect for a target
#
# This function scans the target's source directory for both V1 (.O3DEReflect.xml)
# and V2 (headers with O3DE_CLASS/O3DE_STRUCT macros) files and enables
# code generation for both.
#
# \arg:TARGET - Name of the target
# \arg:INPUT_DIR - Optional input directory (defaults to target source dir)
# \arg:OUTPUT_DIR - Optional output directory for generated files
#
function(ly_enable_o3de_reflect_auto)
    set(options)
    set(oneValueArgs TARGET INPUT_DIR OUTPUT_DIR)
    set(multiValueArgs)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARG_TARGET)
        message(FATAL_ERROR "ly_enable_o3de_reflect_auto requires a TARGET argument")
    endif()

    # Get source directory
    get_target_property(TARGET_SOURCE_DIR ${ARG_TARGET} SOURCE_DIR)
    if(NOT ARG_INPUT_DIR)
        set(ARG_INPUT_DIR "${TARGET_SOURCE_DIR}")
    endif()

    # Enable V1 if XML files exist
    file(GLOB_RECURSE O3DE_REFLECT_XML_FILES "${ARG_INPUT_DIR}/*.O3DEReflect.xml")
    if(O3DE_REFLECT_XML_FILES)
        ly_enable_o3de_reflect(TARGET ${ARG_TARGET} INPUT_DIR "${ARG_INPUT_DIR}")
    endif()

    # For V2, we would need to scan headers for O3DE_CLASS/O3DE_STRUCT macros
    # This is more complex and typically requires explicit listing of files
    # or using a pre-build script to identify them
    # 
    # Users should use ly_enable_o3de_reflect_v2 with explicit HEADERS list
    # for best results
    
endfunction()
