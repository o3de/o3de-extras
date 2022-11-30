# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT

set(FILES
    ../Assets/Editor/Images/Icons/Resources.qrc
    ../Assets/Editor/Images/Icons/ROS_import_icon.svg
    Source/RobotImporter/Pages/CheckAssetPage.cpp
    Source/RobotImporter/Pages/CheckAssetPage.h
    Source/RobotImporter/Pages/CheckUrdfPage.cpp
    Source/RobotImporter/Pages/CheckUrdfPage.h
    Source/RobotImporter/Pages/FileSelectionPage.cpp
    Source/RobotImporter/Pages/FileSelectionPage.h
    Source/RobotImporter/Pages/PrefabMakerPage.cpp
    Source/RobotImporter/Pages/PrefabMakerPage.h
    Source/RobotImporter/Pages/IntroPage.cpp
    Source/RobotImporter/Pages/IntroPage.h
    Source/RobotImporter/RobotImporterWidget.cpp
    Source/RobotImporter/RobotImporterWidget.h
    Source/RobotImporter/RobotImporterWidgetUtils.cpp
    Source/RobotImporter/RobotImporterWidgetUtils.h
    Source/RobotImporter/ROS2RobotImporterEditorSystemComponent.cpp
    Source/RobotImporter/ROS2RobotImporterEditorSystemComponent.h
    Source/RobotImporter/URDF/CollidersMaker.cpp
    Source/RobotImporter/URDF/CollidersMaker.h
    Source/RobotImporter/URDF/InertialsMaker.cpp
    Source/RobotImporter/URDF/InertialsMaker.h
    Source/RobotImporter/URDF/JointsMaker.cpp
    Source/RobotImporter/URDF/JointsMaker.h
    Source/RobotImporter/URDF/PrefabMakerUtils.cpp
    Source/RobotImporter/URDF/PrefabMakerUtils.h
    Source/RobotImporter/URDF/UrdfParser.cpp
    Source/RobotImporter/URDF/UrdfParser.h
    Source/RobotImporter/URDF/URDFPrefabMaker.cpp
    Source/RobotImporter/URDF/URDFPrefabMaker.h
    Source/RobotImporter/URDF/VisualsMaker.cpp
    Source/RobotImporter/URDF/VisualsMaker.h
    Source/RobotImporter/Utils/RobotImporterUtils.cpp
    Source/RobotImporter/Utils/RobotImporterUtils.h
    Source/RobotImporter/Utils/SourceAssetsStorage.cpp
    Source/RobotImporter/Utils/SourceAssetsStorage.h
    Source/RobotImporter/Utils/TypeConversions.cpp
    Source/RobotImporter/Utils/TypeConversions.h
    Source/ROS2EditorSystemComponent.cpp
    Source/ROS2EditorSystemComponent.h
    Source/ROS2GemUtilities.cpp
)
