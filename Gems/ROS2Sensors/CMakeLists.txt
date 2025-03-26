
o3de_gem_setup("ROS2Sensors")

ly_add_external_target_path(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty)

add_subdirectory(Code)
