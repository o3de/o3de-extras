# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

if (PAL_TRAIT_BUILD_TESTS_SUPPORTED)
    if (PAL_TRAIT_TEST_PYTEST_SUPPORTED)
        # PAL_TRAIT_BUILD macros are used by platform detection.
        ly_add_pytest(
                NAME Gem::${gem_name}.PythonTests
                TEST_SUITE smoke
                TEST_SERIAL
                PATH ${CMAKE_CURRENT_LIST_DIR}/SmokeTests_Periodic.py
                RUNTIME_DEPENDENCIES
                    Legacy::Editor
                    AZ::AssetProcessor
                    ${gem_name}.Editor
                COMPONENT
                    SmokeTests
        )
    endif ()
endif ()
