#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

import pytest
from ly_test_tools.o3de.editor_test import EditorTestSuite, EditorSingleTest


@pytest.mark.SUITE_periodic  # Marks the test suite as being part of a Periodic test suite
@pytest.mark.parametrize("launcher_platform", ['linux_editor'])  # This test works on Linux editor
@pytest.mark.parametrize("project", ["AutomatedTesting"])  # Use the AutomatedTesting project
class TestAutomation(EditorTestSuite):
    # Declaring a class that extends from EditorSingleTest declares a single test.
    class SmokeTests_EnterGameModeWorks(EditorSingleTest):
        # This runs a Single Test in a single Editor. For further work check EditorSingleTest siblings
        from .tests import SmokeTests_EnterGameModeWorks as test_module
