if (PAL_TRAIT_BUILD_TESTS_SUPPORTED)
    if (PAL_TRAIT_TEST_PYTEST_SUPPORTED)
        # PAL_TRAIT_BUILD macros are used by platform detection.
        ly_add_pytest(
                NAME SmokeTests_Periodic
                TEST_SUITE periodic
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
