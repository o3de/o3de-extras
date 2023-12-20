@ECHO OFF
REM
REM Copyright (c) Contributors to the Open 3D Engine Project.
REM For complete copyright and license terms please see the LICENSE at the root of this distribution.
REM
REM SPDX-License-Identifier: Apache-2.0 OR MIT
REM
REM

set PROJECT_PATH=%1
set INFO_FILE_PATH=%2
set SOUND_BANK_PATH=%3

REM In order to use cmake/EngineFinder.cmake it must be run from the project folder
pushd %PROJECT_PATH%
cmake  -DWWISE_INFO_FILE_PATH=%INFO_FILE_PATH% -DWWISE_SOUND_BANK_PATH=%SOUND_BANK_PATH% -P %~dp0run_bank_info_parser.cmake
popd
