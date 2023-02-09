
# Query the gem name from the gem.json file if possible
# otherwise fallback to using RosRobotSample
o3de_find_ancestor_gem_root(gem_root_path gem_name "${CMAKE_CURRENT_SOURCE_DIR}")
if (NOT gem_name)
    set(gem_name "RosRobotSample")
endif()

# This will export the path to the directory containing the gem.json
# to the "SourcePaths" entry within the "cmake_dependencies.<project>.assetbuilder.setreg"
# which is generated when cmake is run
# This path is the gem root directory
if(PAL_TRAIT_BUILD_HOST_TOOLS)
    ly_create_alias(NAME ${gem_name}.Builders NAMESPACE Gem)
endif()
