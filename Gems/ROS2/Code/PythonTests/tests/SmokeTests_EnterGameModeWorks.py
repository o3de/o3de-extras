# Test Case Title : Check that entering into gamemode works

# List of results that we want to check, this is not 100% necessary but its a good
# practice to make it easier to debug tests.
# Here we define a tuple of tests

# fmt: off
class Tests:
    enter_game_mode                           = ("Entered game mode",                      "Failed to enter game mode")
    topics_published                          = ("Topics were published during game mode", "Failed to publish topics during game mode")
    topics_not_published_outside_of_game_mode = ("All simulation topics closed",           "Failed to close publishers after simulation")
# fmt: on


def check_result(result, msg):
    from editor_python_test_tools.utils import Report
    if not result:
        Report.result(msg, False)
        raise Exception(msg + " : FAILED")


def check_topics():
    # TODO: Currently this is a workaround
    #       In the final solution environment shall be successfully sourced, investigate why paths are not there
    #       Edit: Looks like PYTHONPATH is not read correctly
    import sys
    sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
    sys.path.append('/opt/ros/humble/local/lib/python3.10/dist-packages')

    import rclpy
    from rclpy.node import Node

    def get_topic_list():
        node_dummy = Node("_ros2cli_dummy_to_show_topic_list")
        topic_list = node_dummy.get_topic_names_and_types()
        node_dummy.destroy_node()
        return topic_list

    topics = []
    rclpy.init()
    topic_list = get_topic_list()

    for info in topic_list:
        topics.append(info[0])
    rclpy.shutdown()
    return topics
def SmokeTest_EnterGameModeWorks():
    # Description: This test checks that entering into game mode works by opening an empty level
    # and entering into the game mode. The is in game mode state should be changed after doing it
    # Next it checks if there are some additional ROS topics published during the game mode

    # Import report and test helper utilities
    from editor_python_test_tools.utils import Report
    from editor_python_test_tools.utils import TestHelper as helper
    # All exposed python bindings are in azlmbr
    import azlmbr.legacy.general as general

    # Required for automated tests
    helper.init_idle()

    # Open the level called "Warehouse".
    # We use a warehouse level for a smoke test - it already has a robot prefab present
    helper.open_level(level="Warehouse", directory='')


    topics_before_game_mode = check_topics()

    # Using the exposed Python API from editor in CryEditPy.py we can enter into game mode this way
    general.enter_game_mode()

    # The script drives the execution of the test, to return the flow back to the editor,
    # we will tick it one time
    general.idle_wait_frames(1)

    # Now we can use the Report.result() to report the state of a result
    # if the second argument is false, it will mark this test as failed, however it will keep going.
    Report.result(Tests.enter_game_mode, general.is_in_game_mode())

    topics_in_game_mode = check_topics()

    Report.result(Tests.topics_published, len(topics_in_game_mode) > len(topics_before_game_mode))

    # Instead of using Report.result(), you can also use:
    # assert is_in_game_mode, "Didn't enter into game mode"
    # However this would stop the test at this point and not report anything when it succeeds

    # The test will end at this point, is good practice to exit game mode or reset any changed stated
    # *DO NOT* close the editor, the editor will close automatically and report the error code
    general.exit_game_mode()

    # this line is needed to update the simulation state
    general.idle_wait_frames(1)

    topics_after_game_mode = check_topics()
    Report.result(Tests.topics_not_published_outside_of_game_mode, len(topics_after_game_mode) == len(topics_before_game_mode))


if __name__ == "__main__":
    # This utility starts up the test and sets up the state for knowing what test is currently being run
    from editor_python_test_tools.utils import Report
    Report.start_test(SmokeTest_EnterGameModeWorks)