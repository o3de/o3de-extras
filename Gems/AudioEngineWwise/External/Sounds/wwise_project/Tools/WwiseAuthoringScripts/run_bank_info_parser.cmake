#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Include the project's EngineFinder so we can find the engine
# This cmake script must be run from the project folder in order to work correctly
# because EngineFinder.cmake expects CMAKE_CURRENT_SOURCE_DIR to be the project folder
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/EngineFinder.cmake)

if(CMAKE_MODULE_PATH)
    cmake_path(SET O3DE_ENGINE_PYTHON_PATH NORMALIZE "${CMAKE_MODULE_PATH}/../python/python.cmd")
    message(STATUS "O3DE engine Python path is  ${O3DE_ENGINE_PYTHON_PATH}")

    cmake_path(SET PYTHON_SCRIPT_PATH NORMALIZE ${CMAKE_CURRENT_SOURCE_DIR}/Sounds/wwise_project/Tools/WwiseAuthoringScripts/bank_info_parser.py)

    execute_process(COMMAND ${O3DE_ENGINE_PYTHON_PATH} "${PYTHON_SCRIPT_PATH}" "${WWISE_INFO_FILE_PATH}" "${WWISE_SOUND_BANK_PATH}")
else()
    message(ERROR "Unable to determine O3DE engine path")
endif()

