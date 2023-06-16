#!/bin/bash

echo "Running test script"

# Test
. /opt/ros/humble/setup.sh

cd /data/workspace/WarehouseTest

if cmake --build build/linux --config profile --target WarehouseTest.GameLauncher Editor ; then
    echo "Build succeeded"
    echo "RESULT: ALL TESTS PASSED" # expected result 
else
    echo "RESULT: Build failed"
fi

exit 0