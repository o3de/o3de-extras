#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

o3de_pal_dir(pal_dir ${CMAKE_CURRENT_LIST_DIR}/Platform/${PAL_PLATFORM_NAME} "${gem_restricted_path}" "${gem_path}" "${gem_parent_relative_path}")

include(${pal_dir}/PAL_${PAL_PLATFORM_NAME_LOWERCASE}.cmake)

if (PAL_TRAIT_AZQTCOMPONENTSFORPYTHON_SUPPORTED)
    # Find the qt package location first so we can add its CMake folder
    # to `CMAKE_PREFIX_PATH` and allow `find_package` to locate Qt6Config.cmake.
    ly_get_package_association("Qt" QT_PACKAGE_NAME)
    ly_package_get_target_folder(${QT_PACKAGE_NAME} QT_DOWNLOAD_LOCATION)
    set(QT_MODULE_DIR ${QT_DOWNLOAD_LOCATION}/${QT_PACKAGE_NAME}/qt)

    # Ensure the Qt package's CMake config directory is searched.
    list(APPEND CMAKE_PREFIX_PATH "${QT_MODULE_DIR}/lib/cmake")

    find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets CONFIG)
                     
    # The library we're creating bindings for.
    set(AZQTCOMPONENTS_LIBRARY "AzQtComponents")
        
    #Define the shared library for which we will create bindings.
    set_property(TARGET ${AZQTCOMPONENTS_LIBRARY} PROPERTY PREFIX "")
    # Ensure Python development headers are available for Shiboken
    find_package(Python3 COMPONENTS Development REQUIRED)
    set(PYTHON_INCLUDE_DIRS ${Python3_INCLUDE_DIRS})
    
    add_shiboken_project(
        NAME AzQtComponentsForPython
        NAMESPACE Gem
        MODULE_NAME azqtpyside
        WRAPPED_HEADER ${CMAKE_CURRENT_LIST_DIR}/bindings_azqtcomponents.h
        TYPESYSTEM_FILE ${CMAKE_CURRENT_LIST_DIR}/bindings_azqtcomponents.xml
        GENERATED_FILES ${CMAKE_CURRENT_LIST_DIR}/generated_files.cmake
        LICENSE_HEADER ${CMAKE_CURRENT_LIST_DIR}/license_header.txt
        DEPENDENCIES 
            ${AZQTCOMPONENTS_LIBRARY}
            Gem::EditorPythonBindings.Static
        INCLUDE_DIRS 
            ${LY_ROOT_FOLDER}/Code/Framework/AzCore 
            ${LY_ROOT_FOLDER}/Code/Framework/AzQtComponents
            ${LY_ROOT_FOLDER}/Code/Framework/AzCore/Platform/${PAL_PLATFORM_NAME} 
            ${PYTHON_INCLUDE_DIRS}
            ${QT_MODULE_DIR}/include
            ${QT_MODULE_DIR}/include/QtWidgets
            ${QT_MODULE_DIR}/include/QtCore
            ${QT_MODULE_DIR}/include/QtGui
    )

    if (TARGET AzQtComponentsForPython.Editor)
        target_compile_options(AzQtComponentsForPython.Editor PRIVATE ${PAL_TRAIT_AZQTCOMPONENTSFORPYTHON_COMPILE_OPTIONS})
    endif()
    
endif()
