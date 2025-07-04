#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(FILES
    ../Assets/Editor/Images/Icons/ROS2RobotImporter.qrc
    ../Assets/Editor/Images/Icons/ToolbarIcon.svg
    Source/Tools/ROS2RobotImporterEditorSystemComponent.cpp
    Source/Tools/ROS2RobotImporterEditorSystemComponent.h
    Source/RobotImporter/FixURDF/FixURDF.cpp
    Source/RobotImporter/FixURDF/FixURDF.h
    Source/RobotImporter/Pages/ModifiedURDFWindow.cpp
    Source/RobotImporter/Pages/ModifiedURDFWindow.h
    Source/RobotImporter/Pages/CheckAssetPage.cpp
    Source/RobotImporter/Pages/CheckAssetPage.h
    Source/RobotImporter/Pages/RobotDescriptionPage.cpp
    Source/RobotImporter/Pages/RobotDescriptionPage.h
    Source/RobotImporter/Pages/FileSelectionPage.cpp
    Source/RobotImporter/Pages/FileSelectionPage.h
    Source/RobotImporter/Pages/PrefabMakerPage.cpp
    Source/RobotImporter/Pages/PrefabMakerPage.h
    Source/RobotImporter/Pages/IntroPage.cpp
    Source/RobotImporter/Pages/IntroPage.h
    Source/RobotImporter/Pages/XacroParamsPage.cpp
    Source/RobotImporter/Pages/XacroParamsPage.h
    Source/RobotImporter/RobotImporterWidget.cpp
    Source/RobotImporter/RobotImporterWidget.h
    Source/RobotImporter/SDFormat/Hooks/ROS2AckermannModelHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2CameraSensorHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2GNSSSensorHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2ImuSensorHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2JointPoseTrajectoryModelHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2JointStatePublisherModelHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2LidarSensorHook.cpp
    Source/RobotImporter/SDFormat/Hooks/ROS2SkidSteeringModelHook.cpp
    Source/RobotImporter/SDFormat/ROS2ModelPluginHooks.h
    Source/RobotImporter/SDFormat/ROS2SDFormatHooksUtils.cpp
    Source/RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h
    Source/RobotImporter/SDFormat/ROS2SensorHooks.h
    Source/RobotImporter/URDF/ArticulationsMaker.cpp
    Source/RobotImporter/URDF/ArticulationsMaker.h
    Source/RobotImporter/URDF/CollidersMaker.cpp
    Source/RobotImporter/URDF/CollidersMaker.h
    Source/RobotImporter/URDF/InertialsMaker.cpp
    Source/RobotImporter/URDF/InertialsMaker.h
    Source/RobotImporter/URDF/JointsMaker.cpp
    Source/RobotImporter/URDF/JointsMaker.h
    Source/RobotImporter/URDF/PrefabMakerUtils.cpp
    Source/RobotImporter/URDF/PrefabMakerUtils.h
    Source/RobotImporter/URDF/RobotControlMaker.cpp
    Source/RobotImporter/URDF/RobotControlMaker.h
    Source/RobotImporter/URDF/SensorsMaker.cpp
    Source/RobotImporter/URDF/SensorsMaker.h
    Source/RobotImporter/URDF/UrdfParser.cpp
    Source/RobotImporter/URDF/UrdfParser.h
    Source/RobotImporter/URDF/URDFPrefabMaker.cpp
    Source/RobotImporter/URDF/URDFPrefabMaker.h
    Source/RobotImporter/URDF/VisualsMaker.cpp
    Source/RobotImporter/URDF/VisualsMaker.h
    Source/RobotImporter/xacro/XacroUtils.cpp
    Source/RobotImporter/xacro/XacroUtils.h
    Source/RobotImporter/Utils/DefaultSolverConfiguration.h
    Source/RobotImporter/Utils/ErrorUtils.cpp
    Source/RobotImporter/Utils/ErrorUtils.h
    Source/RobotImporter/Utils/FilePath.cpp
    Source/RobotImporter/Utils/FilePath.h
    Source/RobotImporter/Utils/RobotImporterUtils.cpp
    Source/RobotImporter/Utils/RobotImporterUtils.h
    Source/RobotImporter/Utils/SourceAssetsStorage.cpp
    Source/RobotImporter/Utils/SourceAssetsStorage.h
    Source/RobotImporter/Utils/TypeConversions.cpp
    Source/RobotImporter/Utils/TypeConversions.h
    Source/SdfAssetBuilder/SdfAssetBuilder.cpp
    Source/SdfAssetBuilder/SdfAssetBuilder.h
    Source/SdfAssetBuilder/SdfAssetBuilderSettings.cpp
    Source/SdfAssetBuilder/SdfAssetBuilderSettings.h
    Source/SdfAssetBuilder/SdfAssetBuilderSystemComponent.cpp
    Source/SdfAssetBuilder/SdfAssetBuilderSystemComponent.h
)

# optional, legacy features compilation
if (WITH_GAZEBO_MSGS)
    list(APPEND FILES
        Source/Spawner/ROS2SpawnerEditorComponent.cpp
        Source/Spawner/ROS2SpawnerEditorComponent.h
        Source/Spawner/ROS2SpawnPointEditorComponent.cpp
        Source/Spawner/ROS2SpawnPointEditorComponent.h
    )
endif ()

