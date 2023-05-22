if (NOT "$ENV{ROS_DISTRO}" STREQUAL "${ROS_DISTRO}")
    message(FATAL_ERROR "ROS 2 distribution does not match. Configuration created for: ${ROS_DISTRO}, sourced distribution: $ENV{ROS_DISTRO}. Remove CMakeCache.txt and reconfigure project.")
endif()