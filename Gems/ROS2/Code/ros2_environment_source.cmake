

if (NOT DEFINED ENV{ROS_DISTRO})
    message(WARNING, "To build ROS2 Gem a ROS distribution needs to be source, but none detected")
endif()
message("Building ROS2 Gem with ros2 $ENV{ROS_DISTRO}")

#TODO - compare to previous env since we need to rerun cmake if we source a different ros2 env!
#TODO - can be done with a file that is in CONFIGURE_DEPENDS so that a change triggers build
if (NOT DEFINED ENV{AMENT_PREFIX_PATH})
    message(WARNING, "$ENV{AMENT_PREFIX_PATH} missing from env. ROS 2 needs to be sourced to build ROS2 Gem")
endif()
set (AMENT_PREFIX_PATH "$ENV{AMENT_PREFIX_PATH}")

# ros2 directories with libraries, e.g. /opt/ros/galactic/lib, locally built custom packages etc.
set(ros2_library_directories)
set(ros2_include_directories)
foreach(ros2_packages_path IN LISTS AMENT_PREFIX_PATH)
    #message("including packages: ${ros2_packages_path}")
    string(REPLACE ":" ";" ros2_packages_path ${ros2_packages_path})
    list(APPEND ros2_library_directories "${ros2_packages_path}/lib")
    list(APPEND ros2_include_directories "${ros2_packages_path}/include")
endforeach()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
set(ros2_libraries ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

#message("Found libraries: ${ros2_libraries}")
#message("Found include directories: ${ros2_include_directories}")
#message("Library directories: ${ros2_library_directories}")