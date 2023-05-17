import pytest
from ly_test_tools import LAUNCHERS
from ly_test_tools.o3de.editor_test import EditorTestSuite, EditorSingleTest

@pytest.mark.SUITE_periodic # Marks the test suite as being part of a Periodic test suite
@pytest.mark.parametrize("launcher_platform", ['windows_editor']) # This test works on Windows editor
@pytest.mark.parametrize("project", ["AutomatedTesting"]) # Use the AutomatedTesting project
class TestAutomation(EditorTestSuite):
    
    # Declaring a class that extends from EditorSingleTest declares a single test. 
    class SmokeTests_EnterGameModeWorks(EditorSingleTest):
        # This sets the class variable test_module to be loaded into a `EnterGameWorks.py` file, run by the Editor as a test.
        from .tests import SmokeTests_EnterGameModeWorks as test_module
