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
    find_package(Qt5 5.12 REQUIRED COMPONENTS Core Gui Widgets)
    
    # Find the qt package location.
    ly_get_package_association("Qt" QT_PACKAGE_NAME)
    ly_package_get_target_folder(${QT_PACKAGE_NAME} QT_DOWNLOAD_LOCATION)
    set(QT_MODULE_DIR ${QT_DOWNLOAD_LOCATION}/${QT_PACKAGE_NAME}/qt)
                     
    # The library we're creating bindings for.
    set(AZQTCOMPONENTS_LIBRARY "AzQtComponents")
        
    #Define the shared library for which we will create bindings.
    set_property(TARGET ${AZQTCOMPONENTS_LIBRARY} PROPERTY PREFIX "")
    
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
            ${QT_MODULE_DIR}/include
            ${QT_MODULE_DIR}/include/QtWidgets
            ${QT_MODULE_DIR}/include/QtCore
            ${QT_MODULE_DIR}/include/QtGui
    )
    
endif()
