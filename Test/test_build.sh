#!/bin/bash

sumOfTests=0
sumOfSuccesfulTests=0

command=""
runTestCommand () {
    sumOfTests=$((sumOfTests+1))
    echo "Running test command: $command"
    eval $command
    if (($? == 0)) ; then
        sumOfSuccesfulTests=$((sumOfSuccesfulTests+1))
    else
        echo "Command $command failed"
    fi
}

summorizeTests () {
    if (($sumOfTests == $sumOfSuccesfulTests)) ; then
        echo "RESULT: ALL TESTS PASSED"
    else
        echo "RESULT: $sumOfSuccesfulTests out of $sumOfTests passed"
    fi
}

echo "Running test script"

# Test
. /opt/ros/humble/setup.sh

cd /data/workspace/WarehouseTest

command="cmake --build build/linux --config profile --target WarehouseTest.GameLauncher Editor"
runTestCommand

cd ..
command="./o3de/python/python.sh -m pytest --build-directory ./WarehouseTest/build/linux/bin/profile/ ./o3de-extras/Gems/ROS2/Code/PythonTests/SmokeTests_Periodic.py"
runTestCommand

summorizeTests
if [ $sumOfTests -eq $sumOfSuccesfulTests ] then
    exit 0
else
    exit 1
fi